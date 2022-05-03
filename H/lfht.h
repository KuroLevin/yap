
#ifndef __LFHT_H__
#define __LFHT_H__

#include <stddef.h>
#include <threads.h>

#ifndef LFHT_DEBUG
#define LFHT_DEBUG 0
#endif

#if LFHT_DEBUG

#define MAX_NODES 2
#define HASH_SIZE 1
#define ROOT_HASH_SIZE HASH_SIZE
//2 works for tests if the hash values never exceed 32 bits
#define MR_FREQUENCY 1
#define MR_THRESHOLD 1
#else

#define MAX_NODES 3
//ROOT_HASH_SIZE + ((WORD_SIZE/4)-1) * HASH_SIZE needs to be >= WORD_SIZE
#define ROOT_HASH_SIZE 20
#define HASH_SIZE 4

#define MR_FREQUENCY 2048
#define MR_THRESHOLD 256

#endif

#define WORD_SIZE 64
#define CACHE_LINE_SIZE 64


struct lfht_head {
	struct lfht_node *entry_hash;
	int root_hnode_size;
	int hnode_size;
	int max_chain_nodes;
	int mr_frequency;
	int mr_threshold;
	size_t (*hash_func)(void *);
	int (*key_cmp)(void *, void *);
	void (*key_free)(void *);
	int max_threads;
	struct mr_entry *thread_array;
	tss_t tss_key;
};


struct lfht_head *init_lfht(
		int max_threads);

struct lfht_head *init_lfht_arbitrary(
		int max_threads,
		size_t (*hash_func)(void *),
		int (*key_cmp)(void *, void *),
		void (*key_free)(void *));

struct lfht_head *init_lfht_explicit(
		int root_hnode_size,
		int hnode_size,
		int max_chain_nodes,
		int mr_frequency,
		int mr_threshold,
		int max_threads,
		size_t (*hash_func)(void *),
		int (*key_cmp)(void *, void *),
		void (*key_free)(void *));

//this function is multithread-unsafe!
void free_lfht_mtu(
		struct lfht_head *head);

void *lfht_search(
		struct lfht_head *head,
		void *key);

int lfht_search_explicit(
		struct lfht_head *head,
		void *key,
		void **result);

void *lfht_insert(
		struct lfht_head *head,
		void *key,
		void *value);

void *lfht_remove(
		struct lfht_head *head,
		void *key);

int lfht_remove_explicit(
		struct lfht_head *head,
		void *key,
		void **result);

void *lfht_find_next_mtu(
		struct lfht_head *head,
		void *key);

//debug interface


void *lfht_debug_search(
		struct lfht_head *head,
		void *key);

#endif // __LFHT_H__

