#include <stdlib.h>
#include <stdint.h>
#include <stdatomic.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/membarrier.h>
#include "lfht.h"


#if LFHT_DEBUG

#include <assert.h>

#endif

enum ntype { HASH, LEAF };

struct lfht_node;

struct lfht_node_hash {
	struct lfht_node * _Atomic array[0];
};

struct kv_pair {
	void *key;
	void *value;
};

struct lfht_node_leaf {
	size_t hash;
	union {
		void *value;
		struct kv_pair *pair;
	};
	struct lfht_node * _Atomic next;
};

struct lfht_node {
	enum ntype type : 1;
	size_t gen      : WORD_SIZE-1;
	union {
		struct lfht_node_hash hash;
		struct lfht_node_leaf leaf;
	};
};

struct lfht_tss {
	int thread_id;
	struct lfht_head *head;
};

struct lfht_node *search_remove(
		size_t hash,
		void *key,
		struct lfht_head *head,
		int thread_id);

struct lfht_node *search_insert(
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		void *value,
		struct lfht_head *head,
		int thread_id);

void expand_hash(
		size_t hash,
		struct lfht_node *prevhnode,
		int prevlvl,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_head *head,
		int thread_id);

void adjust_chain_nodes(
		struct lfht_node *cnode,
		struct lfht_node *prevhnode,
		int prevlvl,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_head *head,
		int thread_id);

void adjust_node(
		struct lfht_node *cnode,
		struct lfht_node *prevhnode,
		int prevlvl,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_head *head,
		int thread_id);

struct lfht_node *search_node(
		size_t hash,
		void *key,
		struct lfht_head *head,
		int thread_id);

struct lfht_node *create_hash_node(
		struct lfht_head *head,
		int size,
		int lvl);

int try_init_thread(struct lfht_head *head);

void end_thread(struct lfht_tss *tss);

void free_all_nodes_mtu(struct lfht_node *hnode, struct lfht_head *head);

struct lfht_node *find_next_mtu(
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		struct lfht_head *head);

//memory reclamation

struct mr_entry {
	struct node_list *tll;
	int count;
	_Atomic size_t hazard_hash;
	_Atomic int hazard_level;
	struct lfht_node * _Atomic hazard_pointers[2];
	atomic_flag claim;
} __attribute__((aligned(CACHE_LINE_SIZE)));

struct mr_entry *init_mr(int max_threads);

void free_mr_mtu(struct mr_entry *array, int max_threads);

int mr_thread_acquire(
		struct mr_entry *array,
		int max_threads);

void mr_thread_release(
		struct mr_entry *array,
		int thread_id);

void mr_update_hl(
		struct mr_entry *array,
		int thread_id,
		int level);

void mr_update_hh(
		struct mr_entry *array,
		int thread_id,
		size_t hash);

struct lfht_node * _Atomic *mr_get_hp_array(
		struct mr_entry *array,
		int thread_id);

void mr_reclaim_node(
		struct lfht_head *head,
		int thread_id,
		struct lfht_node *lfht_node);


// debug search
#if LFHT_DEBUG

void *debug_search_chain(
		struct lfht_node *cnode,
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		struct lfht_head *head);

void *debug_search_hash(
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		struct lfht_head *head);

#endif

//interface

struct lfht_head *init_lfht(
		int max_threads)
{
	return init_lfht_explicit(
			ROOT_HASH_SIZE,
			HASH_SIZE,
			MAX_NODES,
			MR_FREQUENCY,
			MR_THRESHOLD,
			max_threads,
			NULL,
			NULL,
			NULL);
}

struct lfht_head *init_lfht_arbitrary(
		int max_threads,
		size_t (*hash_func)(void *),
		int (*key_cmp)(void *, void *),
		void (*key_free)(void *))
{
	return init_lfht_explicit(
			ROOT_HASH_SIZE,
			HASH_SIZE,
			MAX_NODES,
			MR_FREQUENCY,
			MR_THRESHOLD,
			max_threads,
			hash_func,
			key_cmp,
			key_free);
}

struct lfht_head *init_lfht_explicit(
		int root_hnode_size,
		int hnode_size,
		int max_chain_nodes,
		int mr_frequency,
		int mr_threshold,
		int max_threads,
		size_t (*hash_func)(void *),
		int (*key_cmp)(void *, void *),
		void (*key_free)(void *))
{
	struct lfht_head *head = malloc(sizeof(struct lfht_head));
	if(root_hnode_size > WORD_SIZE)
		root_hnode_size = WORD_SIZE;
	head->root_hnode_size = root_hnode_size;
	if(hnode_size > WORD_SIZE)
		hnode_size = WORD_SIZE;
	head->hnode_size = hnode_size;
	head->max_chain_nodes = max_chain_nodes;
	head->mr_frequency = mr_frequency;
	head->mr_threshold = mr_threshold;
	head->hash_func = hash_func;
	head->key_cmp = key_cmp;
	head->key_free = key_free;
	head->max_threads = max_threads;
	head->thread_array = init_mr(max_threads);
	head->entry_hash = create_hash_node(head, root_hnode_size, 0);
	tss_create(&(head->tss_key), (tss_dtor_t) end_thread);
	return head;
}

void free_lfht_mtu(struct lfht_head *head)
{
	free_all_nodes_mtu(head->entry_hash, head);
	free_mr_mtu(head->thread_array, head->max_threads);
	return;
}

int try_init_thread(struct lfht_head *head)
{
	struct lfht_tss *tss = tss_get(head->tss_key);
	if(tss)
		return tss->thread_id;
	tss = malloc(sizeof(struct lfht_tss));
	tss->head = head;
	tss->thread_id = mr_thread_acquire(head->thread_array, head->max_threads);
	tss_set(head->tss_key, tss);
	syscall(SYS_membarrier, MEMBARRIER_CMD_REGISTER_PRIVATE_EXPEDITED, 0, 0);
	return tss->thread_id;
}

void end_thread(struct lfht_tss *tss)
{
	mr_thread_release(tss->head->thread_array, tss->thread_id);
	free(tss);
}

int lfht_search_explicit(
		struct lfht_head *head,
		void *key,
		void **result)
{
	int thread_id = try_init_thread(head);
	size_t hash;
	if(head->hash_func)
		hash = head->hash_func(key);
	else
		hash = (size_t)key;
	mr_update_hh(
			head->thread_array,
			thread_id,
			hash);
	struct lfht_node *node = search_node(
			hash,
			key,
			head,
			thread_id);
	if(node){
		if(head->hash_func)
			*result = node->leaf.pair->value;
		else
			*result = node->leaf.value;
		return 1;
	}
	else{
		*result = NULL;
		return 0;
	}
}

void *lfht_search(
		struct lfht_head *head,
		void *key)
{
	void *result;
	lfht_search_explicit(head, key, &result);
	return result;
}

void *lfht_insert(
		struct lfht_head *head,
		void *key,
		void *value)
{
	int thread_id = try_init_thread(head);
	size_t hash;
	if(head->hash_func)
		hash = head->hash_func(key);
	else
		hash = (size_t)key;
	mr_update_hh(
			head->thread_array,
			thread_id,
			hash);
	struct lfht_node *node = search_insert(
			head->entry_hash,
			0,
			hash,
			key,
			value,
			head,
			thread_id);
	if(head->hash_func)
		return node->leaf.pair->value;
	else
		return node->leaf.value;
}

int lfht_remove_explicit(
		struct lfht_head *head,
		void *key,
		void **result)
{
	int thread_id = try_init_thread(head);
	size_t hash;
	if(head->hash_func)
		hash = head->hash_func(key);
	else
		hash = (size_t)key;
	mr_update_hh(
			head->thread_array,
			thread_id,
			hash);
	struct lfht_node *node = search_remove(
			hash,
			key,
			head,
			thread_id);
	if(node){
		*result = node->leaf.value;
		mr_reclaim_node(
				head,
				thread_id,
				node);
		return 1;
	}
	else{
		*result = NULL;
		return 0;
	}
}

void *lfht_remove(
		struct lfht_head *head,
		void *key)
{
	void *result;
	lfht_remove_explicit(head, key, &result);
	return result;
}

//TODO: need to find a better way to deal with non pointer type keys
void *lfht_find_next_mtu(
		struct lfht_head *head,
		void *key)
{
	size_t hash;
	if(head->hash_func && key){
		hash = head->hash_func(key);
	} else {
		hash = (size_t)key;
	}
	struct lfht_node *node = find_next_mtu(head->entry_hash, 0, hash, key, head);
	if(node){
		if(head->hash_func)
			return node->leaf.pair->value;
		else
			return node->leaf.value;
	} else {
		return NULL;
	}
}

//debug interface

#if LFHT_DEBUG

void *lfht_debug_search(
		struct lfht_head *head,
		void *key)
{
	size_t hash;
	if(head->hash_func)
		hash = head->hash_func(key);
	else
		hash = (size_t)key;
	return debug_search_hash(head->entry_hash, 0, hash, key, head);
}

#endif

//auxiliary

struct lfht_node *create_leaf_node(
		size_t hash,
		void *key,
		void *value,
		int gen,
		struct lfht_node *next,
		struct lfht_head *head)
{
	struct lfht_node *node = aligned_alloc(
			sizeof(struct lfht_node),
			sizeof(struct lfht_node));
	node->type = LEAF;
	node->gen = gen;
	node->leaf.hash = hash;
	if(head->hash_func){
		node->leaf.pair = malloc(sizeof(struct kv_pair));
		node->leaf.pair->key = key;
		node->leaf.pair->value = value;
	}
	else{
		node->leaf.value = value;
	}
	atomic_init(&(node->leaf.next), next);
	return node;
}

struct lfht_node *create_hash_node(
		struct lfht_head *head,
		int size,
		int lvl)
{
	int fl = head->root_hnode_size + (lvl * head->hnode_size);
	if(fl > WORD_SIZE)
		size = head->hnode_size - (fl - WORD_SIZE);
	struct lfht_node *node = aligned_alloc(
			sizeof(struct lfht_node),
			sizeof(struct lfht_node) + (1<<size)*sizeof(struct lfht_node *));
	node->type = HASH;
	for(int i=0; i < 1<<size; i++){
		atomic_init(&(node->hash.array[i]), node);
	}
	return node;
}

size_t get_bucket_index(size_t hash, struct lfht_head *head, int level)
{
	if(level)
		return (hash >> (WORD_SIZE - head->root_hnode_size - (level * head->hnode_size))) & ((1 << head->hnode_size) - 1);
	else
		return (hash >> (WORD_SIZE - head->root_hnode_size)) & ((1 << head->root_hnode_size) - 1);
}

struct lfht_node * _Atomic *get_bucket(
		size_t hash,
		struct lfht_node *hnode,
		struct lfht_head *head,
		size_t level)
{
	return &(hnode->hash.array[get_bucket_index(hash, head, level)]);
}

struct lfht_node *valid_ptr(struct lfht_node *next)
{
	return (struct lfht_node *) ((uintptr_t) next & ~((WORD_SIZE/2)-1));
}

struct lfht_node *unset_vflag(struct lfht_node *ptr)
{
	return (struct lfht_node *) ((uintptr_t) ptr & ~1);
}

struct lfht_node *set_vflag(struct lfht_node *ptr)
{
	return (struct lfht_node *) ((uintptr_t) ptr | 1);
}

unsigned get_vflag(struct lfht_node *ptr)
{
	return (uintptr_t) ptr & 1;
}

struct lfht_node *unset_hflag(struct lfht_node *ptr)
{
	return (struct lfht_node *) ((uintptr_t) ptr & ~2);
}

struct lfht_node *set_hflag(struct lfht_node *ptr)
{
	return (struct lfht_node *) ((uintptr_t) ptr | 2);
}

unsigned get_hflag(struct lfht_node *ptr)
{
	return (uintptr_t) ptr & 2;
}

struct lfht_node *set_tag(struct lfht_node *ptr, unsigned level)
{
	return (struct lfht_node *) ((uintptr_t) ptr | (level << 1));
}

unsigned get_tag(struct lfht_node *ptr)
{
	return ((uintptr_t) ptr & (((WORD_SIZE/2)-2))) >> 1;
}

unsigned last_level(struct lfht_head *head, int lvl)
{
	return (head->root_hnode_size + (lvl * head->hnode_size)) >= WORD_SIZE ||
	       lvl >= ((WORD_SIZE/4)-1);
}

struct lfht_node *mark_invalid(struct lfht_node * _Atomic *field)
{
	struct lfht_node *expect = unset_vflag(atomic_load_explicit(
				field,
				memory_order_consume));
#if LFHT_DEBUG
	assert(expect);
#endif
	while(!atomic_compare_exchange_weak_explicit(
				field,
				&expect,
				(struct lfht_node *)((uintptr_t) expect | 1),
				memory_order_acq_rel,
				memory_order_consume)){
#if LFHT_DEBUG
		assert(expect);
#endif
		if((uintptr_t) expect & 1)
			return NULL;
	}
	return expect;
}

int force_cas(struct lfht_node *node, struct lfht_node *hash)
{
	struct lfht_node *expect = atomic_load_explicit(
			&(node->leaf.next),
			memory_order_consume);
	do{
#if LFHT_DEBUG
		assert(expect);
#endif
		if(expect == hash)
			return 1;
		else if(get_vflag(expect) || get_tag(expect) >= get_tag(hash))
			return 0;
	}while(!atomic_compare_exchange_weak_explicit(
				&(node->leaf.next),
				&expect,
				hash,
				memory_order_acq_rel,
				memory_order_consume));
	return 1;
}

int hpll_find_node(
		size_t hash,
		void *key,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_node **nodeptr,
		struct lfht_node * _Atomic **last_valid,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node * _Atomic *prev = get_bucket(hash, hnode, head, lvl);
	*nodeptr = atomic_load(prev);
	struct lfht_node *iter = valid_ptr(*nodeptr),
	                 *tmp;
	int cur_hp = 0;
	struct lfht_node * _Atomic *hp = mr_get_hp_array(
			head->thread_array,
			thread_id);
	while(iter != hnode){
		atomic_store(&(hp[cur_hp]), iter);
		if(atomic_load(prev) != *nodeptr)
			return hpll_find_node(
					hash,
					key,
					hnode,
					lvl,
					nodeptr,
					last_valid,
					head,
					thread_id);
		tmp = atomic_load(&(iter->leaf.next));
		if(get_vflag(tmp)){
			if(!atomic_compare_exchange_strong(
						prev,
						nodeptr,
						set_tag(valid_ptr(tmp),get_tag(*nodeptr))))
				return hpll_find_node(
						hash,
						key,
						hnode,
						lvl,
						nodeptr,
						last_valid,
						head,
						thread_id);
		}
		else{
			if(iter->leaf.hash == hash){
				if(!head->key_cmp || !head->key_cmp(key, iter->leaf.pair->key)){
					*nodeptr = iter;
					if(last_valid)
						*last_valid = prev;
					return 1; //TODO ordered list
				}
			}
			prev = &(iter->leaf.next);
			cur_hp ^= 1;
		}
		*nodeptr = unset_vflag(tmp);
		iter = valid_ptr(*nodeptr);
	}
	if(last_valid)
		*last_valid = prev;
	return 0;
}


int find_node(
		size_t hash,
		void *key,
		struct lfht_node *invnode,
		struct lfht_node **hnode,
		int *lvl,
		struct lfht_node **prevhnode,
		int *prevlvl,
		struct lfht_node **nodeptr,
		struct lfht_node * _Atomic **last_valid,
		int *count,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node * _Atomic *bucket = get_bucket(hash, *hnode, head, *lvl),
	                 * _Atomic *prevbucket,
	                 *tmp = atomic_load_explicit(
	                 		bucket,
	                 		memory_order_consume),
	                 *iter = unset_hflag(tmp);
	if(count)
		*count = 0;
	if(*prevhnode){
		prevbucket = get_bucket(hash, *prevhnode, head, *prevlvl);
	}
	if(get_hflag(tmp)){
		*hnode = iter;
		(*lvl)++;
		*prevhnode = NULL;
		*prevlvl = -1;
		return find_node(
				hash,
				key,
				invnode,
				hnode,
				lvl,
				prevhnode,
				prevlvl,
				nodeptr,
				last_valid,
				count,
				head,
				thread_id);
	}
	else if(
			!*prevhnode ||
			(*prevhnode != *hnode &&
			get_hflag(atomic_load_explicit(
					prevbucket,
					memory_order_acquire)))){
		*prevhnode = *hnode;
		*prevlvl = *lvl;
		mr_update_hl(
				head->thread_array,
				thread_id,
				*lvl);
		return find_node(
				hash,
				key,
				invnode,
				hnode,
				lvl,
				prevhnode,
				prevlvl,
				nodeptr,
				last_valid,
				count,
				head,
				thread_id);
	}
	if(last_level(head, *prevlvl)){
		if(invnode){
			hpll_find_node(
					hash,
					key,
					*hnode,
					*lvl,
					nodeptr,
					last_valid,
					head,
					thread_id);
			return 0;
		}
		return hpll_find_node(
				hash,
				key,
				*hnode,
				*lvl,
				nodeptr,
				last_valid,
				head,
				thread_id);
	}
	if(last_valid){
		*last_valid = bucket;
		*nodeptr = iter;
	}
	while(iter != *hnode){
		if(iter->type == HASH){
#if LFHT_DEBUG
			assert(get_tag(tmp) <= *lvl + 1);
			assert(*prevhnode == *hnode);
#endif
			*hnode = iter;
			*lvl = get_tag(tmp);
			return find_node(
					hash,
					key,
					invnode,
					hnode,
					lvl,
					prevhnode,
					prevlvl,
					nodeptr,
					last_valid,
					count,
					head,
					thread_id);
		}
		tmp = atomic_load_explicit(
				&(iter->leaf.next),
				memory_order_consume);
#if LFHT_DEBUG
		assert(tmp);
#endif
		if(!get_vflag(tmp)){
			if(!invnode && iter->leaf.hash == hash){
				if(!head->key_cmp || !head->key_cmp(key, iter->leaf.pair->key)){
					*nodeptr = iter;
					return 1;
				}
			}
			if(last_valid){
				*last_valid = &(iter->leaf.next);
				*nodeptr = tmp;
			}
			if(count)
				(*count)++;
		}
		else if(iter == invnode){
			return 1;
		}
		if(get_tag(tmp) > (*prevlvl + 1)){
			*prevhnode = NULL;
			*prevlvl = -1;
			return find_node(
					hash,
					key,
					invnode,
					hnode,
					lvl,
					prevhnode,
					prevlvl,
					nodeptr,
					last_valid,
					count,
					head,
					thread_id);
		}
		else if(get_tag(tmp) > *prevlvl){
			struct lfht_node *check = atomic_load_explicit(
					prevbucket,
					memory_order_acquire);
			if(get_hflag(check)){
#if LFHT_DEBUG
				assert(*hnode == unset_hflag(check) || *hnode == *prevhnode);
#endif
				*hnode = unset_hflag(check);
				*lvl = *prevlvl + 1;
				*prevhnode = NULL;
				*prevlvl = -1;
				return find_node(
						hash,
						key,
						invnode,
						hnode,
						lvl,
						prevhnode,
						prevlvl,
						nodeptr,
						last_valid,
						count,
						head,
						thread_id);
			}
		}
#if LFHT_DEBUG
		assert(iter != invnode);
#endif
		iter = valid_ptr(tmp);
	}
	return 0;
}

void make_unreachable(
		struct lfht_node *cnode,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_node *prevhnode,
		int prevlvl,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node *tmp = atomic_load_explicit(
	                 		&(cnode->leaf.next),
	                 		memory_order_consume),
	                 *iter = cnode,
	                 *valid_after = NULL,
	                 *check,
	                 * _Atomic *prevbucket = get_bucket(cnode->leaf.hash, prevhnode, head, prevlvl);
#if LFHT_DEBUG
	assert(!last_level(head, prevlvl));
#endif
	void *key = NULL;
	if(head->hash_func)
		key = cnode->leaf.pair->key;
#if LFHT_DEBUG
	assert(tmp);
#endif
	unsigned node_tag = get_tag(tmp);
#if LFHT_DEBUG
	assert(node_tag <= (lvl + 1));
#endif
	while(iter->type == LEAF){
		tmp = atomic_load_explicit(
				&(iter->leaf.next),
				memory_order_consume);
#if LFHT_DEBUG
		assert(tmp);
#endif
		if(get_tag(tmp) > node_tag)
			return;
		if(
				get_tag(tmp) > prevlvl &&
				get_hflag(check = atomic_load_explicit(
						prevbucket,
						memory_order_acquire))){
			hnode = unset_hflag(check);
			lvl = prevlvl + 1;
			if(find_node(
						cnode->leaf.hash,
						key,
						cnode,
						&hnode,
						&lvl,
						&prevhnode,
						&prevlvl,
						NULL,
						NULL,
						NULL,
						head,
						thread_id))
				make_unreachable(
						cnode,
						hnode,
						lvl,
						prevhnode,
						prevlvl,
						head,
						thread_id);
			return;
		}
		if(!valid_after && !get_vflag(tmp))
			valid_after = iter;
		iter = valid_ptr(tmp);
	}
	if(!valid_after)
		valid_after = iter;
	hnode=iter;
	lvl = get_tag(tmp);
	struct lfht_node * _Atomic *valid_before = NULL,
	                 *valid_before_next = NULL;
	if(find_node(
			cnode->leaf.hash,
			key,
			cnode,
			&hnode,
			&lvl,
			&prevhnode,
			&prevlvl,
			&valid_before_next,
			&valid_before,
			NULL,
			head,
			thread_id)){
		if(atomic_compare_exchange_strong_explicit(
					valid_before,
					&valid_before_next,
					set_tag(valid_after, get_tag(valid_before_next)),
					memory_order_acq_rel,
					memory_order_consume))
			return;
		else
			return make_unreachable(
					cnode,
					hnode,
					lvl,
					prevhnode,
					prevlvl,
					head,
					thread_id);
	}
	return;
}

//remove functions

struct lfht_node *search_remove(
		size_t hash,
		void *key,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node *cnode = NULL,
	                 *hnode = head->entry_hash,
	                 *prevhnode = NULL,
	                 *next,
	                 * _Atomic *last_valid,
	                 *found;
	int lvl = 0,
	    prevlvl = -1;
	if(find_node(
				hash,
				key,
				NULL,
				&hnode,
				&lvl,
				&prevhnode,
				&prevlvl,
				&cnode,
				&last_valid,
				NULL,
				head,
				thread_id)){
		found = cnode;
		if((next = mark_invalid(&(cnode->leaf.next)))){
			if(get_tag(next) > prevlvl)
				if(!find_node(
							hash,
							key,
							cnode,
							&hnode,
							&lvl,
							&prevhnode,
							&prevlvl,
							NULL,
							NULL,
							NULL,
							head,
							thread_id))
					return found;
			if(last_level(head, prevlvl)){
				if(get_bucket(hash, prevhnode, head, prevlvl) !=last_valid){
					cnode = set_tag(cnode, prevlvl);
					next = unset_vflag(next);
				}
				else {
					next = valid_ptr(next);
				}
				if(!atomic_compare_exchange_strong(
							last_valid,
							&cnode,
							unset_vflag(next))){
					hpll_find_node(
							hash,
							key,
							hnode,
							lvl,
							&cnode,
							&last_valid,
							head,
							thread_id);
				}
			}
			else{
				make_unreachable(
						cnode,
						hnode,
						lvl,
						prevhnode,
						prevlvl,
						head,
						thread_id);
			}
			return found;
		}
	}
	return NULL;
}

//insertion functions

struct lfht_node *search_insert(
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		void *value,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node *cnode,
	                 *prevhnode = NULL,
	                 * _Atomic *last_valid;
	int prevlvl = -1,
	    count;
	if(find_node(
				hash,
				key,
				NULL,
				&hnode,
				&lvl,
				&prevhnode,
				&prevlvl,
				&cnode,
				&last_valid,
				&count,
				head,
				thread_id))
		return cnode;
	if(prevhnode != hnode){
		expand_hash(
				hash,
				prevhnode,
				prevlvl,
				hnode,
				lvl,
				head,
				thread_id);
		mr_update_hl(
				head->thread_array,
				thread_id,
				lvl);
		prevhnode = hnode;
		prevlvl = lvl;
	}
	if(count >= head->max_chain_nodes){
		struct lfht_node *new_hash = create_hash_node(head, head->hnode_size, lvl + 1);
		if(atomic_compare_exchange_strong_explicit(
					last_valid,
					&cnode,
					set_tag(new_hash, lvl + 1),
					memory_order_acq_rel,
					memory_order_consume)){
			expand_hash(
					hash,
					hnode,
					lvl,
					new_hash,
					lvl + 1,
					head,
					thread_id);
			return search_insert(
					new_hash,
					lvl + 1,
					hash,
					key,
					value,
					head,
					thread_id);
		}
		else{
			free(new_hash);
		}
	}
	else{
		struct lfht_node *new_node = create_leaf_node(
				hash,
				key,
				value,
				lvl,
				set_tag(hnode, lvl),
				head);
		if(atomic_compare_exchange_strong_explicit(
					last_valid,
					&cnode,
					set_tag(new_node, get_tag(cnode)),
					memory_order_acq_rel,
					memory_order_consume))
			return new_node;
		else
			free(new_node);
	}
	return search_insert(
			hnode,
			lvl,
			hash,
			key,
			value,
			head,
			thread_id);
}

//expansion functions

void expand_hash(
		size_t hash,
		struct lfht_node *prevhnode,
		int prevlvl,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node * _Atomic *prevbucket = get_bucket(hash, prevhnode, head, prevlvl),
	                 *tmp = atomic_load_explicit(
	                 		prevbucket,
	                 		memory_order_consume);
	if(!get_hflag(tmp)){
		adjust_chain_nodes(
				tmp,
				prevhnode,
				prevlvl,
				hnode,
				lvl,
				head,
				thread_id);
		atomic_store_explicit(
				prevbucket,
				set_hflag(hnode),
				memory_order_release);
	}
}

void adjust_chain_nodes(
		struct lfht_node *cnode,
		struct lfht_node *prevhnode,
		int prevlvl,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node * _Atomic *prevbucket = get_bucket(cnode->leaf.hash, prevhnode, head, prevlvl),
	                 *tmp = atomic_load_explicit(
	                 		&(cnode->leaf.next),
	                 		memory_order_consume),
	                 *next = valid_ptr(tmp);
#if LFHT_DEBUG
	assert(tmp);
#endif
	if(
			(get_tag(tmp) >= lvl && next != hnode) ||
			get_hflag(atomic_load_explicit(
					prevbucket,
					memory_order_acquire)))
		return;
	if(next != hnode)
		adjust_chain_nodes(
				next,
				prevhnode,
				prevlvl,
				hnode,
				lvl,
				head,
				thread_id);
	if(!get_vflag(tmp)){
		if(get_hflag(atomic_load_explicit(
						prevbucket,
						memory_order_acquire)))
			return;
		if(!force_cas(cnode, set_tag(hnode, lvl)))
			return;
		adjust_node(
				cnode,
				prevhnode,
				prevlvl,
				hnode,
				lvl,
				head,
				thread_id);
	}
	return;
}

void adjust_node(
		struct lfht_node *cnode,
		struct lfht_node *prevhnode,
		int prevlvl,
		struct lfht_node *hnode,
		int lvl,
		struct lfht_head *head,
		int thread_id)
{
	int counter = 0;
	struct lfht_node * _Atomic *current_valid = get_bucket(cnode->leaf.hash, hnode, head, lvl),
	                 *expected_value = atomic_load_explicit(
	                 		current_valid,
	                 		memory_order_consume),
	                 *iter = expected_value,
	                 * _Atomic *prevbucket = get_bucket(cnode->leaf.hash, prevhnode, head, prevlvl);
	if(get_hflag(expected_value))
		return;
	if(get_hflag(atomic_load_explicit(
					prevbucket,
					memory_order_acquire)))
		return;
	while(iter != cnode && iter->type == LEAF){
		struct lfht_node *tmp = atomic_load_explicit(
				&(iter->leaf.next),
				memory_order_consume);
#if LFHT_DEBUG
		assert(tmp);
#endif
		if(get_tag(tmp) != lvl)
			return;
		if(get_hflag(atomic_load_explicit(
						prevbucket,
						memory_order_acquire)))
			return;
		if(!get_vflag(tmp)){
			current_valid = &(iter->leaf.next);
			expected_value = tmp;
			counter++;
		}
		iter = valid_ptr(tmp);
	}
	if(iter == cnode)
		return;
#if LFHT_DEBUG
	assert(iter == hnode);
#endif
	if(counter >= head->max_chain_nodes){
#if LFHT_DEBUG
		assert(get_vflag(atomic_load(&(cnode->leaf.next))));
#endif
		return;
	}
	if(set_tag(hnode, lvl) != atomic_load_explicit(
				&(cnode->leaf.next),
				memory_order_acquire))
		return;
	if(atomic_compare_exchange_strong_explicit(
				current_valid,
				&expected_value,
				set_tag(cnode, get_tag(expected_value)),
				memory_order_acq_rel,
				memory_order_consume)){
		if(
				set_vflag(set_tag(hnode, lvl)) ==
				atomic_load_explicit(
					&(cnode->leaf.next),
					memory_order_acquire)){
			make_unreachable(
					cnode,
					hnode,
					lvl,
					prevhnode,
					prevlvl,
					head,
					thread_id);
		}
		return;
	}
	return adjust_node(
			cnode,
			prevhnode,
			prevlvl,
			hnode,
			lvl,
			head,
			thread_id);
}

struct lfht_node *find_next_chain_mtu(
		struct lfht_node *node,
		size_t hash,
		void *key,
		struct lfht_head *head)
{
	struct lfht_node *found = NULL;
	while(node->type == LEAF){
		if(node->leaf.hash >= hash &&
				(!key ||
				node->leaf.hash != hash ||
				(head->key_cmp && head->key_cmp(key, node->leaf.pair->key) < 0))) {
			if(
					!found ||
					found->leaf.hash > node->leaf.hash ||
						(found->leaf.hash == node->leaf.hash &&
						head->key_cmp &&
						head->key_cmp(found->leaf.pair->key, node->leaf.pair->key)>0)){
				found = node;
			}
		}
		node = valid_ptr(atomic_load_explicit(&(node->leaf.next), memory_order_relaxed));
	}
	return found;
}

struct lfht_node *find_next_mtu(
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		struct lfht_head *head)
{
	size_t size;
	if(lvl)
		size = 1 << head->hnode_size;
	else
		size = 1 << head->root_hnode_size;
	for(size_t i = get_bucket_index(hash, head, lvl); i < size; i++){
		struct lfht_node *node = valid_ptr(atomic_load_explicit(&(hnode->hash.array[i]), memory_order_relaxed));
		if(node != hnode && node->type == HASH){
			node = find_next_mtu(node, lvl + 1, hash, key, head);
			if(node)
				return node;
		}
		else if(node->type == LEAF){
			node = find_next_chain_mtu(node, hash, key, head);
			if(node)
				return node;
		}
		hash = 0;
	}
	return NULL;
}

void free_chain_mtu(struct lfht_node *node)
{
	struct lfht_node *next = valid_ptr(atomic_load_explicit(&(node->leaf.next), memory_order_relaxed));
	if(next->type == LEAF)
		free_chain_mtu(next);
	free(node);
}

void free_all_nodes_mtu(struct lfht_node *hnode, struct lfht_head *head)
{
	size_t size;
	if(hnode == head->entry_hash)
		size = 1 << head->root_hnode_size;
	else
		size = 1 << head->hnode_size;
	for(size_t i = 0; i < size; i++){
		struct lfht_node *node = valid_ptr(atomic_load_explicit(&(hnode->hash.array[i]), memory_order_relaxed));
		if(node == hnode)
			continue;
		else if(node->type == HASH)
			free_all_nodes_mtu(node, head);
		else
			free_chain_mtu(node);
	}
	free(hnode);
}

// searching functions

struct lfht_node *search_node(
		size_t hash,
		void *key,
		struct lfht_head *head,
		int thread_id)
{
	struct lfht_node *cnode,
	                 *hnode = head->entry_hash,
	                 *prevhnode = NULL;
	int lvl = 0,
	    prevlvl = -1;
	if(find_node(
				hash,
				key,
				NULL,
				&hnode,
				&lvl,
				&prevhnode,
				&prevlvl,
				&cnode,
				NULL,
				NULL,
				head,
				thread_id))
		return cnode;
	else
		return NULL;
}

//memory reclamation functions

struct node_list {
	struct lfht_node *node;
	struct node_list *next;
};

struct hazard_array {
	size_t hash;
	int level;
	struct lfht_node *hp[2];
	int count;
};

int match_hash(unsigned hl, size_t hh, size_t nh, struct lfht_head *head)
{
	return !((nh ^ hh) & (~0ULL << (WORD_SIZE - (hl * head->hnode_size + head->root_hnode_size))));
}

struct mr_entry *init_mr(int max_threads)
{
	struct mr_entry *array = aligned_alloc(
			CACHE_LINE_SIZE,
			max_threads * sizeof(struct mr_entry));
	for(int i=0; i<max_threads; i++){
		atomic_flag_clear(&(array[i].claim));
		array[i].tll = NULL;
		array[i].count = 0;
		atomic_init(&(array[i].hazard_hash), 0);
		atomic_init(&(array[i].hazard_level), -1);
	}
	return array;
}

void free_mr_mtu(struct mr_entry *array, int max_threads)
{
	for(int i=0; i<max_threads; i++){
		while(array[i].tll){
			struct node_list *next = array[i].tll->next;
			free(array[i].tll);
			array[i].tll = next;
		}
	}
	free(array);
}


int mr_thread_acquire(
		struct mr_entry *array,
		int max_threads)
{
	int i = 0;
	while(1){
		if(!atomic_flag_test_and_set(&(array[i].claim)))
			return i;
		i = (i+1) % max_threads;
	}
}

void mr_thread_release(
		struct mr_entry *array,
		int thread_id)
{
	atomic_flag_clear(&(array[thread_id].claim));
}

void mr_update_hl(
		struct mr_entry *array,
		int thread_id,
		int level)
{
	atomic_store_explicit(
			&(array[thread_id].hazard_level),
			level,
			memory_order_relaxed);
	atomic_signal_fence(memory_order_seq_cst);
}

void mr_update_hh(
		struct mr_entry *array,
		int thread_id,
		size_t hash)
{
	atomic_store_explicit(
			&(array[thread_id].hazard_hash),
			hash,
			memory_order_relaxed);
	atomic_signal_fence(memory_order_seq_cst);
}

struct lfht_node * _Atomic *mr_get_hp_array(
		struct mr_entry *array,
		int thread_id)
{
	return array[thread_id].hazard_pointers;
}

void mr_reclaim_node(
		struct lfht_head *head,
		int thread_id,
		struct lfht_node *lfht_node)
{
	struct node_list *new = malloc(sizeof(struct node_list));
	new->node = lfht_node;
	new->next = head->thread_array[thread_id].tll;
	head->thread_array[thread_id].tll = new;
	head->thread_array[thread_id].count++;
	if(head->thread_array[thread_id].count >= head->mr_frequency){
		head->thread_array[thread_id].count = 0;
		struct hazard_array *ha = malloc(2 * head->max_threads * sizeof(struct hazard_array));
		syscall(SYS_membarrier, MEMBARRIER_CMD_PRIVATE_EXPEDITED, 0, 0);
		for(int i = 0; i < head->max_threads; i++){
			ha[i].hash = atomic_load_explicit(
					&(head->thread_array[i].hazard_hash),
					memory_order_seq_cst);
			ha[i].level = atomic_load_explicit(
					&(head->thread_array[i].hazard_level),
					memory_order_seq_cst);
			ha[i].hp[0] = atomic_load_explicit(
					&(head->thread_array[i].hazard_pointers[0]),
					memory_order_seq_cst);
			ha[i].hp[1] = atomic_load_explicit(
					&(head->thread_array[i].hazard_pointers[1]),
					memory_order_seq_cst);
			ha[i].count = 0;
		}
		syscall(SYS_membarrier, MEMBARRIER_CMD_PRIVATE_EXPEDITED, 0, 0);
		for(int i = 0; i < head->max_threads; i++){
			ha[i + head->max_threads].hash = atomic_load_explicit(
					&(head->thread_array[i].hazard_hash),
					memory_order_seq_cst);
			ha[i + head->max_threads].level = atomic_load_explicit(
					&(head->thread_array[i].hazard_level),
					memory_order_seq_cst);
			ha[i + head->max_threads].hp[0] = atomic_load_explicit(
					&(head->thread_array[i].hazard_pointers[0]),
					memory_order_seq_cst);
			ha[i + head->max_threads].hp[1] = atomic_load_explicit(
					&(head->thread_array[i].hazard_pointers[1]),
					memory_order_seq_cst);
			ha[i + head->max_threads].count = 0;
		}
		struct node_list **ptr = &(head->thread_array[thread_id].tll);
		while(*ptr){
			unsigned gen = (*ptr)->node->gen;
			unsigned tag = get_tag(atomic_load_explicit(&((*ptr)->node->leaf.next), memory_order_relaxed));
#if LFHT_DEBUG
			assert(tag >=gen);
#endif
			size_t hash = (*ptr)->node->leaf.hash;
			struct node_list *tmp;
			int reclaim = 1;
			for(int i = 0; i < 2 * head->max_threads; i++){
				if(head->root_hnode_size + ha[i].level * head->hnode_size >= WORD_SIZE){
					if(ha[i].hp[0] == (*ptr)->node || ha[i].hp[1] == (*ptr)->node){
						reclaim = 0;
						break;
					}
				}
				else if(
						gen <= ha[i].level &&
						tag >= ha[i].level &&
						match_hash(ha[i].level, ha[i].hash, hash, head)){
					reclaim = 0;
					ha[i].count++;
					break;
				}
			}
			if(reclaim){
				tmp = *ptr;
				*ptr = (*ptr)->next;
				atomic_store(&(tmp->node->leaf.next), NULL);
				if(head->hash_func){
					if(head->key_free)
						head->key_free(tmp->node->leaf.pair->key);
					free(tmp->node->leaf.pair);
				}
				free(tmp->node);
				free(tmp);
			}
			else{
				ptr = &((*ptr)->next);
			}
		}
		for(int i = 0; i < 2 * head->max_threads; i++){
			if(ha[i].count > head->mr_threshold){
				struct lfht_node *hnode = head->entry_hash,
						 *prevhnode = NULL,
						 *leaf_node = NULL,
						 * _Atomic *last_valid = NULL;
				int lvl = 0,
				    prevlvl = -1;
				int count;
				find_node(
						ha[i].hash,
						NULL,
						(struct lfht_node *)head, //some address != 0 and != from any leaf node
						&hnode,
						&lvl,
						&prevhnode,
						&prevlvl,
						&leaf_node,
						&last_valid,
						&count,
						head,
						thread_id);
				if(lvl > ha[i].level || last_level(head, lvl))
					continue;
				if(hnode != prevhnode){
					expand_hash(
							ha[i].hash,
							prevhnode,
							prevlvl,
							hnode,
							lvl,
							head,
							thread_id);
					i--;
					continue;
				}
				struct lfht_node *new_hash = create_hash_node(
						head,
						head->hnode_size,
						lvl + 1);
				if(count > 0){
					if(!atomic_compare_exchange_strong_explicit(
								last_valid,
								&leaf_node,
								set_tag(new_hash, lvl + 1),
								memory_order_acq_rel,
								memory_order_relaxed)){
						free(new_hash);
						i--;
						continue;
					}
					expand_hash(
							ha[i].hash,
							hnode,
							lvl,
							new_hash,
							lvl + 1,
							head,
							thread_id);
				}
				else{
					if(!atomic_compare_exchange_strong_explicit(
								last_valid,
								&leaf_node,
								set_hflag(new_hash),
								memory_order_acq_rel,
								memory_order_relaxed)){
						free(new_hash);
						i--;
						continue;
					}
				}
			}
		}
		free(ha);
	}
}


// debug searching

#if LFHT_DEBUG

void *debug_search_hash(
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		struct lfht_head *head)
{
	struct lfht_node * _Atomic *bucket = get_bucket(hash, hnode, head, lvl),
	                 *next_node = atomic_load_explicit(
	                 		bucket,
	                 		memory_order_seq_cst);
	assert(
			(get_hflag(next_node) && unset_hflag(next_node)->type == HASH) || 
			(!get_hflag(next_node) && next_node->type == LEAF) ||
			next_node == hnode);
	next_node = unset_hflag(next_node);
	if(next_node == hnode)
		return NULL;
	else if(next_node->type == HASH)
		return debug_search_hash(next_node, lvl + 1, hash, key, head);
	else
		return debug_search_chain(next_node, hnode, lvl, hash, key, head);
}

void *debug_search_chain(
		struct lfht_node *cnode,
		struct lfht_node *hnode,
		int lvl,
		size_t hash,
		void *key,
		struct lfht_head *head)
{
	if(cnode->leaf.hash == hash){
		if(!head->key_cmp || !head->key_cmp(key, cnode->leaf.pair->key)){
			assert(!get_vflag(atomic_load_explicit(
							&(cnode->leaf.next),
							memory_order_seq_cst)));
			if(head->hash_func)
				return cnode->leaf.pair->value;
			else
				return cnode->leaf.value;
		}
	}
	struct lfht_node *next_node = atomic_load_explicit(
	                 		&(cnode->leaf.next),
	                 		memory_order_seq_cst);
	assert(get_tag(next_node) == lvl);
	next_node = valid_ptr(next_node);
	if(next_node == hnode){
		return NULL;
	} else {
		assert(next_node->type == LEAF);
		return debug_search_chain(next_node, hnode, lvl, hash, key, head);
	}
}

#endif
