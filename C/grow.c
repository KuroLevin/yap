/*************************************************************************
 *									 *
 *	 YAP Prolog 							 *
 *									 *
 *	Yap Prolog was developed at NCCUP - Universidade do Porto	 *
 *									 *
 * Copyright L.Damas, V.S.Costa and Universidade do Porto 1985-1997	 *
 *									 *
 **************************************************************************
 *									 *
 * File:		grow.c							 *
 * Last rev:	Thu Feb 23 1989		vv				 *
 * mods:									 *
 * comments:	Shifting the stacks       				 *
 *									 *
 *************************************************************************/

#include "absmi.h"
#include "YapHeap.h"
#include "yapio.h"
#include "alloc.h"
#include "sshift.h"
#include "YapCompile.h"
#include "attvar.h"
#include "cut_c.h"
#if HAVE_STRING_H
#include <string.h>
#endif
#if YAPOR_THREADS
#include "opt.mavar.h"
#endif /* YAPOR_THREADS */

#if !HAVE_STRNCAT
#define strncat(s0,s1,sz)   strcat(s0,s1)
#endif

typedef enum {
	      STACK_SHIFTING = 0,
	      STACK_COPYING = 1,
	      STACK_INCREMENTAL_COPYING = 2
} what_stack_copying;



static Int p_growheap( USES_REGS1 );
static Int p_growstack( USES_REGS1 );
static Int p_inform_trail_overflows( USES_REGS1 );
static Int p_inform_heap_overflows( USES_REGS1 );
static Int p_inform_stack_overflows( USES_REGS1 );

/* #define undf7  */
/* #define undf5 */

static int growstack(size_t CACHE_TYPE);
static void MoveGlobal( CACHE_TYPE1 );
static void MoveLocalAndTrail( CACHE_TYPE1 );
static void SetHeapRegs(bool CACHE_TYPE);
static void AdjustTrail(bool, bool CACHE_TYPE);
static void AdjustLocal(bool CACHE_TYPE);
static void AdjustGlobal(Int, bool CACHE_TYPE);
static void AdjustGrowStack( CACHE_TYPE1 );
static int  static_growheap(size_t,bool,struct intermediates *,tr_fr_ptr *, TokEntry **, VarEntry ** CACHE_TYPE);
static void cpcellsd(CELL *, CELL *, CELL);
static CELL AdjustAppl(CELL CACHE_TYPE);
static CELL AdjustPair(CELL CACHE_TYPE);
static void AdjustStacksAndTrail(Int, bool CACHE_TYPE);
static void AdjustRegs(int CACHE_TYPE);
static Term AdjustGlobTerm(Term CACHE_TYPE);

static void show_stacks(void)
{
  CACHE_REGS
  fprintf(stderr, "%%       Local+Heap(Global): %8ld cells (%8ldB, address range %p-%p)\n", (unsigned long int)(LCL0-(CELL *)LOCAL_GlobalBase), (unsigned long int)sizeof(CELL)*(LCL0-(CELL *)LOCAL_GlobalBase),LCL0,LOCAL_GlobalBase);
  fprintf(stderr, "%%              Local uses %8ld cells (%8ldB, address range %p-%p)\n", (unsigned long int)(LCL0-ASP), (unsigned long int)(LCL0-ASP)*sizeof(CELL),LCL0,ASP);
  fprintf(stderr, "%%              Heap uses %8ld cells (%p-%p)\n", (unsigned long int)(HR-(CELL *)LOCAL_GlobalBase),(CELL *)LOCAL_GlobalBase,HR);
  fprintf(stderr, "%%       Trail: %8ld markers ((%8ldB, address range%p-%p)\n",
	  (unsigned long int)(LOCAL_TrailTop-LOCAL_TrailBase),(unsigned long int)(LOCAL_TrailTop-LOCAL_TrailBase)*sizeof(tr_fr_ptr*),LOCAL_TrailBase,LOCAL_TrailTop);
  fprintf(stderr, "%%              Trail uses %8ld markers ((%8ldB, address range%p-%p)\n",
	  (unsigned long int)(TR-(tr_fr_ptr)LOCAL_TrailBase),(unsigned long int)(TR-(tr_fr_ptr)LOCAL_TrailBase)*sizeof(tr_fr_ptr*),LOCAL_TrailBase,TR);
}

static void
LeaveGrowMode(prolog_exec_mode grow_mode)
{
  CACHE_REGS
    LOCAL_PrologMode &= ~grow_mode;
}


static void
cpcellsd(register CELL *Dest, register CELL *Org, CELL NOf)
{
#if HAVE_MEMMOVE
  memmove((void *)Dest, (void *)Org, NOf*sizeof(CELL));
#else
  register Int    n_of = NOf;
  for (; n_of >= 0; n_of--)
    *Dest++ = *Org++;
#endif
}


static void
SetHeapRegs(bool copying_threads USES_REGS)
{
#ifdef undf7
  Sfprintf(stderr,"HeapBase = %x\tHeapTop=%x\nGlobalBase=%x\tGlobalTop=%x\nLocalBase=%x\tLocatTop=%x\n", Yap_HeapBase, HeapTop, LOCAL_GlobalBase, H, LCL0, ASP);
#endif
  /* The old stack pointers */
  LOCAL_OldLCL0 = LCL0;
  LOCAL_OldASP = ASP;
  LOCAL_OldGlobalBase = (CELL *)LOCAL_GlobalBase;
  LOCAL_OldH = HR;
  LOCAL_OldH0 = H0;
  LOCAL_OldTrailBase = LOCAL_TrailBase;
  LOCAL_OldTrailTop = LOCAL_TrailTop;
  LOCAL_OldTR = TR;
  LOCAL_OldHeapBase = Yap_HeapBase;
  LOCAL_OldHeapTop = HeapTop;
  /* Adjust stack addresses */
  LOCAL_TrailBase = TrailAddrAdjust(LOCAL_TrailBase);
  LOCAL_TrailTop = TrailAddrAdjust(LOCAL_TrailTop);
  CurrentTrailTop = (tr_fr_ptr)(LOCAL_TrailTop-MinTrailGap);
  LOCAL_GlobalBase = BaseAddrAdjust(LOCAL_GlobalBase);
  LOCAL_LocalBase = LocalAddrAdjust(LOCAL_LocalBase);
#if !USE_SYSTEM_MALLOC && !USE_DL_MALLOC
  AuxSp = PtoBaseAdjust(AuxSp);
  AuxTop = (ADDR)PtoBaseAdjust((CELL *)AuxTop);
#endif
#if !USE_SYSTEM_MALLOC
  if (HeapLim)
    HeapLim = BaseAddrAdjust(HeapLim);
#endif
  /* The registers pointing to one of the stacks */
  if (ENV)
    ENV = PtoLocAdjust(ENV);
  if (ASP)
    ASP = PtoLocAdjust(ASP);
  if (H0)
    H0 = PtoGloAdjust(H0);
  if (LCL0)
    LCL0 = PtoLocAdjust(LCL0);
  if (HR)
    HR = PtoGloAdjust(HR);
  if (Yap_REGS.CUT_C_TOP)
    Yap_REGS.CUT_C_TOP = CutCAdjust(Yap_REGS.CUT_C_TOP);
  if (HB)
    HB = PtoGloAdjust(HB);
  if (LOCAL_OpenArray)
    LOCAL_OpenArray = PtoGloAdjust(LOCAL_OpenArray);
  if (B)
    B = ChoicePtrAdjust(B);
#ifdef YAPOR_THREADS
  {
    choiceptr cpt;
    cpt = Get_LOCAL_top_cp();
    if (cpt) {
      //      cpt = ChoicePtrAdjust( cpt );
      Set_LOCAL_top_cp( cpt );
    }
  }
#endif
#ifdef TABLING
  if (B_FZ)
    B_FZ = ChoicePtrAdjust(B_FZ);
  if (BB)
    BB = ChoicePtrAdjust(BB);
  if (H_FZ)
    H_FZ = PtoGloAdjust(H_FZ);
  if (TR_FZ)
    TR_FZ = PtoTRAdjust(TR_FZ);
#endif /* TABLING */
  if (TR)
    TR = PtoTRAdjust(TR);
  if (YENV)
    YENV = PtoLocAdjust(YENV);
  if (IsOldGlobalPtr(S))
    S = PtoGloAdjust(S);
  else if (IsOldLocalPtr(S))
    S = PtoLocAdjust(S);
  if (!copying_threads) {
    if (LOCAL_GlobalArena)
      LOCAL_GlobalArena = AbsAppl(PtoGloAdjust(RepAppl(LOCAL_GlobalArena)));
  }
  if (LOCAL_AttsMutableList)
    LOCAL_AttsMutableList = AbsAppl(PtoGloAdjust(RepAppl(LOCAL_AttsMutableList)));
  if (LOCAL_WokenGoals)
    LOCAL_WokenGoals = AbsAppl(PtoGloAdjust(RepAppl(LOCAL_WokenGoals)));
  LOCAL_GcGeneration = AbsAppl(PtoGloAdjust(RepAppl(LOCAL_GcGeneration)));
  LOCAL_GcPhase = AbsAppl(PtoGloAdjust(RepAppl(LOCAL_GcPhase)));
}

static void
MoveLocalAndTrail( USES_REGS1 )
{
  /* cpcellsd(To,From,NOfCells) - copy the cells downwards  */
#if USE_SYSTEM_MALLOC
  cpcellsd(ASP, (CELL *)((char *)LOCAL_OldASP+LOCAL_BaseDiff), (CELL *)LOCAL_OldTR - LOCAL_OldASP);
#else
  cpcellsd(ASP, LOCAL_OldASP, (CELL *)LOCAL_OldTR - LOCAL_OldASP);
#endif
}

#ifdef YAPOR_THREADS

static void
CopyLocalAndTrail( USES_REGS1 )
{
  /* cpcellsd(To,From,NOfCells) - copy the cells downwards  */
#if USE_SYSTEM_MALLOC
  cpcellsd((void *)ASP, (void *)LOCAL_OldASP, (CELL *)LOCAL_OldTR - LOCAL_OldASP);
#endif
}

static void
IncrementalCopyStacksFromWorker( USES_REGS1 )
{
  memcpy((void *) PtoGloAdjust((CELL *)LOCAL_start_global_copy),
	 (void *) (LOCAL_start_global_copy),
	 (size_t) (LOCAL_end_global_copy - LOCAL_start_global_copy));
  memcpy((void *) PtoLocAdjust((CELL *)LOCAL_start_local_copy),
	 (void *) LOCAL_start_local_copy,
	 (size_t) (LOCAL_end_local_copy - LOCAL_start_local_copy));
  memcpy((void *) PtoTRAdjust((tr_fr_ptr)LOCAL_start_trail_copy),
	 (void *) (LOCAL_start_trail_copy),
	 (size_t) (LOCAL_end_trail_copy - LOCAL_start_trail_copy));
}

#ifndef TABLING
static CELL
worker_p_binding(int worker_p, CELL *aux_ptr)
{
  CACHE_REGS
    if (aux_ptr > HR) {
      CELL reg = REMOTE_ThreadHandle(worker_p).current_yaam_regs->LCL0_[aux_ptr-LCL0];
      reg = AdjustGlobTerm(reg PASS_REGS);
      return reg;
    } else {
      CELL reg = REMOTE_ThreadHandle(worker_p).current_yaam_regs-> H0_[aux_ptr-H0];
      reg = AdjustGlobTerm(reg PASS_REGS);
      return reg;
    }
}
#endif

static void
RestoreTrail(int worker_p USES_REGS)
{
  tr_fr_ptr aux_tr;

  /* install fase --> TR and LOCAL_top_cp->cp_tr are equal */
  aux_tr = ((choiceptr) LOCAL_start_local_copy)->cp_tr;
  TR = ((choiceptr) LOCAL_end_local_copy)->cp_tr;
  if (TR == aux_tr)
    return;
  if (aux_tr < TR){
    Yap_Error(SYSTEM_ERROR_INTERNAL, TermNil, "oops");
  }
  Yap_NEW_MAHASH((ma_h_inner_struct *)HR PASS_REGS);
  while (TR != aux_tr) {
    CELL aux_cell = TrailTerm(--aux_tr);
    if (IsVarTerm(aux_cell)) {
      if (aux_cell < LOCAL_start_global_copy ||
	  EQUAL_OR_YOUNGER_CP((choiceptr)LOCAL_end_local_copy, (choiceptr)aux_cell)) {
	YAPOR_ERROR_CHECKING((CELL *)aux_cell < H0, "RestoreTrail: aux_cell < H0");
	YAPOR_ERROR_CHECKING((ADDR)aux_cell > LOCAL_LocalBase, "RestoreTrail: aux_cell > LocalBase");
#ifdef TABLING
	*((CELL *) aux_cell) = TrailVal(aux_tr);
#else
	*((CELL *) aux_cell) = worker_p_binding(worker_p, CellPtr(aux_cell));
#endif /* TABLING */
      }
#ifdef TABLING
    } else if (IsPairTerm(aux_cell)) {
      /* avoid frozen segments */
      aux_cell = (CELL) RepPair(aux_cell);
      if (IN_BETWEEN(LOCAL_TrailBase, aux_cell, LOCAL_TrailTop)) {
	aux_tr = (tr_fr_ptr) aux_cell;
      }
#endif /* TABLING */
#ifdef MULTI_ASSIGNMENT_VARIABLES
    } else if (IsApplTerm(aux_cell)) {
      CELL *cell_ptr = RepAppl(aux_cell);
      if (((CELL *)aux_cell < Get_LOCAL_top_cp()->cp_h ||
	   EQUAL_OR_YOUNGER_CP(Get_LOCAL_top_cp(), (choiceptr)aux_cell)) &&
	  !Yap_lookup_ma_var(cell_ptr PASS_REGS)) {
	/* first time we found the variable, let's put the new value */
#ifdef TABLING
	*cell_ptr = TrailVal(aux_tr);
#else
	*cell_ptr = worker_p_binding(worker_p, cell_ptr);
#endif /* TABLING */
      }
      /* skip the old value */
      aux_tr--;
#endif /* MULTI_ASSIGNMENT_VARIABLES */
    }
  }
}

#endif /* YAPOR_THREADS */

static void
MoveGlobal( USES_REGS1 )
{
  /*
   * cpcellsd(To,From,NOfCells) - copy the cells downwards - in
   * absmi.asm
   */
  cpcellsd((CELL *)LOCAL_GlobalBase, (CELL *)LOCAL_OldGlobalBase, LOCAL_OldH - (CELL *)LOCAL_OldGlobalBase);
}

static void
MoveExpandedGlobal( USES_REGS1 )
{
  /*
   * cpcellsd(To,From,NOfCells) - copy the cells downwards - in
   * absmi.asm
   */
  cpcellsd((CELL *)(LOCAL_GlobalBase+(LOCAL_GDiff-LOCAL_BaseDiff)), (CELL *)LOCAL_GlobalBase, LOCAL_OldH - (CELL *)LOCAL_OldGlobalBase);
}

static void
MoveGlobalWithHole( USES_REGS1 )
{
  /*
   * cpcellsd(To,From,NOfCells) - copy the cells downwards - in
   * absmi.asm
   */
#if USE_SYSTEM_MALLOC
  cpcellsd((CELL *)((char *)LOCAL_GlobalBase+(LOCAL_GDiff0-LOCAL_BaseDiff)), (CELL *)LOCAL_GlobalBase, LOCAL_OldH - (CELL *)LOCAL_OldGlobalBase);
#else
  cpcellsd((CELL *)((char *)LOCAL_OldGlobalBase+LOCAL_GDiff0), (CELL *)LOCAL_OldGlobalBase, LOCAL_OldH - (CELL *)LOCAL_OldGlobalBase);
#endif
}

static void
MoveHalfGlobal(CELL *OldPt, size_t ncells USES_REGS)
{
  /*
   * cpcellsd(To,From,NOfCells) - copy the cells downwards - in
   * absmi.asm
   */
  CELL *NewPt = (CELL *)((char*)OldPt+LOCAL_GDiff);
  CELL *IntPt = (CELL *)((char*)OldPt+LOCAL_GDiff0);
  cpcellsd(NewPt, IntPt, ncells);
}

static inline CELL
AdjustAppl(register CELL t0 USES_REGS)
{
  register CELL   *t = RepAppl(t0);

  if (IsOldGlobalPtr(t))
    return (AbsAppl(PtoGloAdjust(t)));
  else if (IsOldTrailPtr(t))
    return (AbsAppl(CellPtoTRAdjust(t)));
  else if (IsHeapP(t))
    return (AbsAppl(CellPtoHeapAdjust(t)));
#ifdef DEBUG
  else {
    /* strange cell */
    /*    Sfprintf(stderr,"% garbage appl %lx found in stacks by stack shifter\n", t0);*/
  }
#endif
  return(t0);
}

static inline CELL
AdjustPair(register CELL t0 USES_REGS)
{
  register CELL  *t = RepPair(t0);

  if (IsOldGlobalPtr(t))
    return (AbsPair(PtoGloAdjust(t)));
  if (IsOldLocalPtr(t))
    return (AbsPair(PtoLocAdjust(t)));
  if (IsOldTrailPtr(t))
    return (AbsPair(CellPtoTRAdjust(t)));
  else if (IsHeapP(t))
    return (AbsPair(CellPtoHeapAdjust(t)));
#ifdef DEBUG
  /* Sfprintf(stderr,"% garbage pair %lx found in stacks by stack shifter\n", t0);*/
#endif
  return(t0);
}

static void
AdjustTrail(bool adjusting_heap, bool thread_copying USES_REGS)
{
  volatile tr_fr_ptr ptt, tr_base = (tr_fr_ptr)LOCAL_TrailBase;

#if defined(YAPOR_THREADS)
  if (thread_copying == STACK_INCREMENTAL_COPYING) {
    ptt =  (tr_fr_ptr)(LOCAL_end_trail_copy);
    tr_base =  (tr_fr_ptr)(LOCAL_start_trail_copy);
  } else {
#else
    ptt = TR;
    tr_base = (tr_fr_ptr)LOCAL_TrailBase;
#endif
#if defined(YAPOR_THREADS)
1  }
#endif
  /* moving the trail is simple, yeaahhh! */
  while (ptt != tr_base) {
    register CELL reg = TrailTerm(ptt-1);
#ifdef FROZEN_STACKS
    register CELL reg2 = TrailVal(ptt-1);
#endif

    ptt--;
    if (IsVarTerm(reg)) {
      if (IsOldLocalInTR(reg))
	TrailTerm(ptt) = LocalAdjust(reg);
      else if (IsOldGlobal(reg))
	TrailTerm(ptt) = GlobalAdjust(reg);
      else if (IsOldTrail(reg))
	TrailTerm(ptt) = TrailAdjust(reg);
      else if (thread_copying) {
	RESET_VARIABLE(&TrailTerm(ptt));
      }
    } else if (IsPairTerm(reg)) {
      TrailTerm(ptt) = AdjustPair(reg PASS_REGS);
#ifdef MULTI_ASSIGNMENT_VARIABLES /* does not work with new structures */
    } else if (IsApplTerm(reg)) {
      TrailTerm(ptt) = AdjustAppl(reg PASS_REGS);
#endif
    }
#ifdef FROZEN_STACKS
    if (IsVarTerm(reg2)) {
      if (IsOldLocal(reg2))
	TrailVal(ptt) = LocalAdjust(reg2);
      else if (IsOldGlobal(reg2))
	TrailVal(ptt) = GlobalAdjust(reg2);
      else if (IsOldTrail(reg2))
	TrailVal(ptt) = TrailAdjust(reg2);
    } else if (IsApplTerm(reg2)) {
      TrailVal(ptt) = AdjustAppl(reg2 PASS_REGS);
    } else if (IsPairTerm(reg2)) {
      TrailVal(ptt) = AdjustPair(reg2 PASS_REGS);
    }
#endif
  }

}

static void
fixPointerCells(CELL *pt, CELL *pt_bot, bool thread_copying USES_REGS)
{
  while (pt > pt_bot) {
    CELL reg = *--pt;
    if (IsVarTerm(reg)) {
      if (IsOldLocal(reg))
	*pt = LocalAdjust(reg);
      else if (IsOldGlobal(reg))
	*pt = GlobalAdjust(reg);
      else if (IsOldTrail(reg))
	*pt = TrailAdjust(reg);
      else if (IsOldCode(reg))
	*pt = CodeAdjust(reg);
    } else if (IsApplTerm(reg)) {
      *pt = AdjustAppl(reg PASS_REGS);
    } else if (IsPairTerm(reg)) {
      *pt = AdjustPair(reg PASS_REGS);
    }
  }
}


#ifdef TABLING
static void
fix_tabling_info( USES_REGS1 )
{
  /* we must fix the dependency frames and the subgoal frames, as they are
     pointing back to the global stack. */
  struct dependency_frame *df;
  struct subgoal_frame *sg;

  df = LOCAL_top_dep_fr;
  while (df) {
    if (DepFr_backchain_cp(df))
      DepFr_backchain_cp(df) = ChoicePtrAdjust(DepFr_backchain_cp(df));
    if (DepFr_leader_cp(df))
      DepFr_leader_cp(df) = ChoicePtrAdjust(DepFr_leader_cp(df));
    if (DepFr_cons_cp(df))
      DepFr_cons_cp(df) = ConsumerChoicePtrAdjust(DepFr_cons_cp(df));
    df = DepFr_next(df);
  }
  sg = LOCAL_top_sg_fr;
  while (sg) {
    if (SgFr_gen_cp(sg))
      SgFr_gen_cp(sg) = GeneratorChoicePtrAdjust(SgFr_gen_cp(sg));
    sg = SgFr_next(sg);
  }
}
#endif /* TABLING */


static void
AdjustSlots(bool thread_copying USES_REGS)
{
  CELL *pt = LOCAL_SlotBase+LOCAL_CurSlot;
  CELL *pt_bot = LOCAL_SlotBase;
  fixPointerCells( pt, pt_bot, thread_copying PASS_REGS);
}

static void
AdjustLocal(bool thread_copying USES_REGS)
{
  register CELL    *pt, *pt_bot;

  /* Adjusting the local */
#if defined(YAPOR_THREADS)
  if (thread_copying == STACK_INCREMENTAL_COPYING) {
    pt =  (CELL *) (LOCAL_end_local_copy);
    pt_bot =  (CELL *) (LOCAL_start_local_copy);
  } else {
#endif
    pt = LCL0;
    pt_bot = ASP;
#if defined(YAPOR_THREADS)
  }
#endif
#ifdef TABLING
  fix_tabling_info( PASS_REGS1 );
#endif /* TABLING */
  fixPointerCells( pt, pt_bot, thread_copying PASS_REGS);
  AdjustSlots( thread_copying PASS_REGS);
}

static Term
AdjustGlobTerm(Term reg USES_REGS)
{
  if (IsVarTerm(reg)) {
    if (IsOldGlobal(reg))
      return GlobalAdjust(reg);
    else if (IsOldLocal(reg))
      return LocalAdjust(reg);
#ifdef MULTI_ASSIGNMENT_VARIABLES
    else if (IsOldTrail(reg))
      return TrailAdjust(reg);
#endif
  } else if (IsApplTerm(reg))
    return AdjustAppl(reg PASS_REGS);
  else if (IsPairTerm(reg))
    return AdjustPair(reg PASS_REGS);
  return AtomTermAdjust(reg);
}

static void
AdjustGlobal(Int sz, bool thread_copying USES_REGS)
{
  ArrayEntry *al = LOCAL_DynamicArrays;
  ArrayEntry *sal = LOCAL_StaticArrays;
  GlobalEntry *gl = LOCAL_GlobalVariables;

  while (al) {
    al->ValueOfDynamicVE = AdjustGlobTerm(al->ValueOfDynamicVE PASS_REGS);
    al = al->NextAE;
  }
  while (gl) {
    if (IsVarTerm(gl->global) ||
	!IsAtomOrIntTerm(gl->global)) {
      gl->global = AdjustGlobTerm(gl->global PASS_REGS);
    }
    gl = gl->NextGE;
  }
  while (sal) {
    if (sal->ArrayType == array_of_nb_terms) {
      UInt arity = -sal->ArrayEArity, i;
      for (i=0; i < arity; i++) {
	/*	    sal->ValueOfVE.lterms[i].tlive = AdjustGlobTerm(sal->ValueOfVE.lterms[i].tlive); */
	Term tlive  = sal->ValueOfStaticVE.lterms[i].tlive;
	if (!IsVarTerm(tlive) || !IsUnboundVar(&sal->ValueOfStaticVE.lterms[i].tlive)) {
	  sal->ValueOfStaticVE.lterms[i].tlive = AdjustGlobTerm(sal->ValueOfStaticVE.lterms[i].tlive PASS_REGS);
	}
      }
    }
    sal = sal->NextAE;
  }

  /*
   * to clean the global now that functors are just variables pointing to
   * the code
   */
#if defined(YAPOR_THREADS)
  if (thread_copying == STACK_INCREMENTAL_COPYING) {
    hpt =  (CELL *) (LOCAL_start_global_copy);
    hpt_max =  (CELL *) (LOCAL_end_global_copy);
  } else {
#endif
    CELL *hpt = LOCAL_OldH0;
#if defined(YAPOR_THREADS)
  }
#endif
  hpt = H0;

  while (hpt < HR) {
    CELL reg;
    reg = *hpt;
    if (IsVarTerm(reg)) {
      if (IsOldGlobal(reg))
	*hpt = GlobalAdjust(reg);
      else if (IsOldLocal(reg))
	*hpt = LocalAdjust(reg);
      else if (IsOldTrail(reg)) {
	*hpt = TrailAdjust(reg);
      } else if ( IsExtensionFunctor((Functor)reg) && reg > 0 && reg % sizeof(CELL)==0 ) {
	ssize_t bigsz =  SizeOfOpaqueTerm(hpt,reg);
	if (bigsz <= 0 || hpt + bigsz > HR ||!IsAtomTerm(hpt[bigsz-1])) {
	  *hpt = reg;
	} else {
	hpt += (bigsz-1);
	*hpt = MkAtomTerm((Atom)GlobalAddrAdjust((ADDR)AtomOfTerm(*hpt)));
	}
      }
    } else if (IsApplTerm(reg)){
      *hpt = AdjustAppl(reg PASS_REGS);
    }else if (IsPairTerm(reg)){
      *hpt = AdjustPair(reg PASS_REGS);
    }else if (IsAtomTerm(reg)) {
      *hpt = AtomTermAdjust(reg);
    }
    hpt++;
  }
}

/*
 * When growing the stack we need to adjust: the local stack cells pointing
 * to the local; the local stack cells and the X terms pointing to the global
 * (just once) the trail cells pointing both to the global and to the local
 */
static void
AdjustStacksAndTrail(Int sz, bool copying_threads USES_REGS)
{
  AdjustTrail(true, copying_threads PASS_REGS);
  AdjustLocal(copying_threads PASS_REGS);
  AdjustGlobal(sz, copying_threads PASS_REGS);
}

void
Yap_AdjustStacksAndTrail(void)
{
  CACHE_REGS
    AdjustStacksAndTrail(0, FALSE PASS_REGS);
}

/*
 * When growing the stack we need to adjust: the local cells pointing to the
 * local; the trail cells pointing to the local
 */
static void
AdjustGrowStack( USES_REGS1 )
{
  AdjustTrail(FALSE, STACK_SHIFTING PASS_REGS);
  AdjustLocal(STACK_SHIFTING PASS_REGS);
}

static void
AdjustRegs(int n USES_REGS)
{
  int            i;
  CELL	       reg;

  for (i = 1; i < n; ++i) {
    reg = (CELL) XREGS[i];
    if (IsVarTerm(reg)) {
      if (IsOldLocal(reg))
	reg = LocalAdjust(reg);
      else if (IsOldGlobal(reg))
	reg = GlobalAdjust(reg);
      else if (IsOldTrail(reg))
	reg = TrailAdjust(reg);
      else if (IsOldCode(reg))
	reg = CodeAdjust(reg);
    } else if (IsApplTerm(reg))
      reg = AdjustAppl(reg PASS_REGS);
    else if (IsPairTerm(reg))
      reg = AdjustPair(reg PASS_REGS);
    XREGS[i] = (Term) reg;
  }
}

static void
AdjustVarTable(VarEntry *ves USES_REGS)
{
  ves->VarAdr = TermNil;
  if (ves->VarRight != NULL) {
    if (IsOldVarTableTrailPtr(ves->VarRight)) {
      ves->VarRight = (VarEntry *)TrailAddrAdjust((ADDR)(ves->VarRight));
    }
    AdjustVarTable(ves->VarRight PASS_REGS);
  }
  if (ves->VarLeft != NULL) {
    if (IsOldVarTableTrailPtr(ves->VarLeft)) {
      ves->VarLeft = (VarEntry *)TrailAddrAdjust((ADDR)(ves->VarLeft));
    }
    AdjustVarTable(ves->VarLeft PASS_REGS);
  }
}

/*
  If we have to shift while we are scanning we need to adjust all
  pointers created by the scanner (Tokens and Variables)
*/
static void
AdjustScannerStacks(TokEntry **tksp, VarEntry **vep USES_REGS)
{
  TokEntry *tks = *tksp;
  VarEntry *ves = *vep;

  if (tks != NULL) {
    if (IsOldTokenTrailPtr(tks)) {
      tks = *tksp = TokEntryAdjust(tks);
    }
  }
  while (tks != NULL) {
    TokEntry *tktmp;

    switch (tks->Tok) {
    case Number_tok:
      if (IsApplTerm(tks->TokInfo)) {
	tks->TokInfo = AdjustAppl(tks->TokInfo PASS_REGS);
      }
      break;
    case Var_tok:
    case String_tok:
    case BQString_tok:
      if (IsOldTrail(tks->TokInfo))
	tks->TokInfo = TrailAdjust(tks->TokInfo);
      break;
    case Name_tok:
      tks->TokInfo = (Term)AtomAdjust((Atom)(tks->TokInfo));
      break;
    default:
      break;
    }
    tktmp = tks->TokNext;
    if (tktmp != NULL) {
      if (IsOldTokenTrailPtr(tktmp)) {
	tktmp = TokEntryAdjust(tktmp);
	tks->TokNext = tktmp;
      }
    }
    tks = tktmp;
  }
  if (ves != NULL) {
    if (IsOldVarTableTrailPtr(ves))
      ves = *vep = (VarEntry *)TrailAddrAdjust((ADDR)ves);
    AdjustVarTable(ves PASS_REGS);
  }
  ves = LOCAL_AnonVarTable;
  if (ves != NULL) {
    if (IsOldVarTableTrailPtr(ves))
      ves = LOCAL_AnonVarTable = VarEntryAdjust(ves);
  }
  while (ves != NULL) {
    VarEntry *vetmp = ves->VarLeft;
    if (vetmp != NULL) {
      if (IsOldVarTableTrailPtr(vetmp)) {
	vetmp = VarEntryAdjust(vetmp);
      }
      ves->VarLeft = vetmp;
    }
    ves->VarAdr = TermNil;
    ves = vetmp;
  }
}

void
Yap_AdjustRegs(int n)
{
  CACHE_REGS
    AdjustRegs(n PASS_REGS);
}

/* Used by do_goal() when we're short of heap space */
static int
static_growheap(size_t esize, bool fix_code, struct intermediates *cip, tr_fr_ptr *old_trp, TokEntry **tksp, VarEntry **vep USES_REGS)
{
  Int size = esize;
  UInt start_growth_time, growth_time;
  int gc_verbose;
  size_t minimal_request = 0L;

  /* adjust to a multiple of 256) */
  if (size < YAP_ALLOC_SIZE)
    size = YAP_ALLOC_SIZE;
  size = AdjustPageSize(size);
  LOCAL_ErrorMessage = NULL;
  if (!Yap_ExtendWorkSpace(size)) {
    Int min_size = AdjustPageSize(((CELL)LOCAL_TrailTop-(CELL)LOCAL_GlobalBase)+MinHeapGap);

    LOCAL_ErrorMessage = NULL;
    if (size < min_size) size = min_size;
    minimal_request = size;
    size = Yap_ExtendWorkSpaceThroughHole(size);
    if (size < 0) {
      LOCAL_ErrorMessage = "Database crashed against Stacks";
      return FALSE;
    }
  }
  start_growth_time = Yap_cputime();
  gc_verbose = Yap_is_gc_verbose();
  LOCAL_heap_overflows++;
  if (gc_verbose) {
#if  defined(YAPOR_THREADS)
    fprintf( stderr, "%% Worker Id %d:\n", worker_id);
#endif
    fprintf( stderr, "%% Database Overflow %d\n", LOCAL_heap_overflows);
    fprintf( stderr, "%%   growing the heap " Int_FORMAT " bytes\n", size);
  }
  /* CreepFlag is set to force heap expansion */
  if ( Yap_only_has_signal( YAP_CDOVF_SIGNAL) ) {
    CalculateStackGap( PASS_REGS1 );
  }
  LOCAL_TrDiff = LOCAL_LDiff = LOCAL_GDiff = LOCAL_GDiff0 = LOCAL_DelayDiff = LOCAL_BaseDiff = size;
  LOCAL_XDiff = LOCAL_HDiff = 0;
  LOCAL_GSplit = NULL;
  YAPEnterCriticalSection();
  SetHeapRegs(FALSE PASS_REGS);
  MoveLocalAndTrail( PASS_REGS1 );
  if (fix_code) {
    CELL *SaveOldH = LOCAL_OldH;
    LOCAL_OldH = (CELL *)cip->freep;
    MoveGlobal( PASS_REGS1 );
    LOCAL_OldH = SaveOldH;
  } else {
    MoveGlobal( PASS_REGS1 );
  }
  if (old_trp) {
    tr_fr_ptr nTR;

    AdjustScannerStacks(tksp, vep PASS_REGS);
    nTR = TR;
    *old_trp = PtoTRAdjust(*old_trp);
    TR = *old_trp;
    AdjustStacksAndTrail(0, FALSE PASS_REGS);
    TR = nTR;
  } else {
    AdjustStacksAndTrail(0, FALSE PASS_REGS);
  }
  AdjustRegs(MaxTemps PASS_REGS);
  if (minimal_request)
    Yap_AllocHole(minimal_request, size);
  YAPLeaveCriticalSection();
  growth_time = Yap_cputime()-start_growth_time;
  LOCAL_total_heap_overflow_time += growth_time;
  if (gc_verbose) {
    fprintf(stderr, "%%   took %g sec\n", (double)growth_time/1000);
    fprintf(stderr, "%% Total of %g sec expanding Database\n", (double)LOCAL_total_heap_overflow_time/1000);
  }
  return(TRUE);
}

/* Used when we're short of heap, usually because of an overflow in
   the attributed stack, but also because we allocated a zone  */
static int
static_growglobal(size_t request, CELL **ptr, CELL *hsplit USES_REGS)
{
  size_t cells_to_move = HR-hsplit;
  UInt start_growth_time, growth_time;
  int gc_verbose;
  char *omax = (char *)H0;
  ADDR old_GlobalBase = LOCAL_GlobalBase;
  UInt minimal_request = 0L;
  Int size = request;
  char vb_msg1 = '\0', *vb_msg2;
  bool do_grow = true;
  /*
    request is the amount of memory we requested, in bytes;
    base_move is the shift in global stacks we had to do
    size is how much space we allocate: it's negative if we just expand
    the delay stack.
    do_grow is whether we expand stacks
  */

  if (hsplit) {
    /* just a little bit of sanity checking */
    if (hsplit < H0 && hsplit > (CELL *)LOCAL_GlobalBase) {
      /* expanding attributed variables */
      if (omax - size > LOCAL_GlobalBase+4096*sizeof(CELL)) {
	/* we can just ask for more room */
	size = 0;
	do_grow = FALSE;
      }
    } else if (hsplit < (CELL*)omax ||
	       hsplit > HR) {
      //Yap_ThrowError(INTERNAL_SYSTEM_ERROR, MkAddressTerm(hsplit), NULL );
    }
    else if (hsplit == (CELL *)omax)
      hsplit = NULL;
    if (size < 0 ||
	(Unsigned(HR)+size < Unsigned(ASP)-StackGap( PASS_REGS1 ) &&
	 hsplit > H0)) {
      /* don't need to expand stacks */
      do_grow = FALSE;
    }
  } else {
    if (Unsigned(HR)+size < Unsigned(ASP)-CreepFlag) {
      /* we can just ask for more room */
      return size;    }
  }
  if (do_grow) {
    if (size < YAP_ALLOC_SIZE)
      size = YAP_ALLOC_SIZE;
    size = AdjustPageSize(size);
  }
  /* adjust to a multiple of 256) */
  LOCAL_ErrorMessage = NULL;
  LOCAL_PrologMode |= GrowStackMode;
  start_growth_time = Yap_cputime();
  if (do_grow) {
    if (!GLOBAL_AllowGlobalExpansion) {
      LOCAL_ErrorMessage = "Global Stack crashed against Local Stack";
      LeaveGrowMode(GrowStackMode);
      return 0;
    }
    if (!GLOBAL_AllowGlobalExpansion || !Yap_ExtendWorkSpace(size)) {
      /* always fails when using malloc */
      LOCAL_ErrorMessage = NULL;
      size += AdjustPageSize(((CELL)LOCAL_TrailTop-(CELL)LOCAL_GlobalBase)+MinHeapGap);
      minimal_request = size;
      size = Yap_ExtendWorkSpaceThroughHole(size);
      if (size < 0) {
	LOCAL_ErrorMessage = "Global Stack crashed against Local Stack";
	LeaveGrowMode(GrowStackMode);
	return 0;
      }
    }
  }
  gc_verbose = Yap_is_gc_verbose();
  LOCAL_delay_overflows++;
  if (gc_verbose) {
    if (hsplit) {
      if (hsplit > H0) {
	vb_msg1 = 'H';
	vb_msg2 = "Global Variable Space";
      } else {
	vb_msg1 = 'D';
	vb_msg2 = "Global Variable Delay Space";
      }
    } else {
      vb_msg1 = 'D';
      vb_msg2 = "Delay";
    }
#if  defined(YAPOR_THREADS)
    fprintf(stderr, "%% Worker Id %d:\n", worker_id);
#endif
    fprintf(stderr, "%% %cO %s Overflow %d\n", vb_msg1, vb_msg2, LOCAL_delay_overflows);
    fprintf(stderr, "%% %cO   growing the stacks " UInt_FORMAT " bytes\n", vb_msg1, size);
  }
  YAPEnterCriticalSection();
  /* we always shift the local and the stack by the same amount */
  if (do_grow) {
    /* we got over a hole */
    if (minimal_request) {
      /* we went over a hole */
      LOCAL_BaseDiff = size+((CELL)LOCAL_TrailTop-(CELL)LOCAL_GlobalBase)-minimal_request;
      LOCAL_LDiff = LOCAL_TrDiff = size;
    } else {
      /* we may still have an overflow */
      LOCAL_BaseDiff = LOCAL_GlobalBase - old_GlobalBase;
      /* if we grow, we need to move the stacks */
      LOCAL_LDiff = LOCAL_TrDiff = LOCAL_BaseDiff+size;
    }
  } else {
    /* stay still */
    LOCAL_LDiff = LOCAL_TrDiff = 0;
    LOCAL_BaseDiff = 0;
  }
  /* now, remember we have delay -- global with a hole in delay or a
     hole in global */
  if (!hsplit) {
    if (!do_grow) {
      LOCAL_DelayDiff = LOCAL_GDiff = LOCAL_GDiff0 = size;
      request = 0L;
    } else {
      /* expand delay stack */
      LOCAL_DelayDiff = LOCAL_GDiff = LOCAL_GDiff0 = LOCAL_LDiff;
    }
  } else {
    /* we want to expand a hole for the delay stack */
    LOCAL_GDiff0 = LOCAL_DelayDiff = LOCAL_BaseDiff;
    LOCAL_GDiff = LOCAL_BaseDiff+request;
  }
  LOCAL_XDiff = LOCAL_HDiff = 0;
  LOCAL_GSplit = hsplit;
  LOCAL_GlobalBase = old_GlobalBase;
  SetHeapRegs(FALSE PASS_REGS);
  if (do_grow) {
    MoveLocalAndTrail( PASS_REGS1 );
    if (hsplit) {
      MoveGlobalWithHole( PASS_REGS1 );
    } else {
      MoveExpandedGlobal( PASS_REGS1 );
    }
  } else if (!hsplit) {
    MoveExpandedGlobal( PASS_REGS1 );
  }
  /* don't run through garbage */
  if (hsplit && (LOCAL_OldH != hsplit)) {
   AdjustStacksAndTrail(request, FALSE PASS_REGS);
  } else {
    AdjustStacksAndTrail(0, FALSE PASS_REGS);
  }
  AdjustRegs(MaxTemps PASS_REGS);
  if (ptr) {

    if (IsOldGlobalPtr(*ptr))
      *ptr = PtoGloAdjust(*ptr);
    else
      *ptr = PtoLocAdjust(*ptr);
  }
  if (hsplit) {
    MoveHalfGlobal(hsplit, cells_to_move PASS_REGS);
    LOCAL_GSplit = hsplit+LOCAL_GDiff0/sizeof(CELL);
  }
  YAPLeaveCriticalSection();
  if (minimal_request) {
    Yap_AllocHole(minimal_request, size);
  }
  growth_time = Yap_cputime()-start_growth_time;
  LOCAL_total_delay_overflow_time += growth_time;
  if (gc_verbose) {
    fprintf(stderr, "%% %cO   took %g sec\n", vb_msg1, (double)growth_time/1000);
    fprintf(stderr, "%% %cO Total of %g sec expanding stacks \n", vb_msg1, (double)LOCAL_total_delay_overflow_time/1000);
  }
  LeaveGrowMode(GrowStackMode);
  if (hsplit) {
    return request;
  } else
    return LOCAL_GDiff-LOCAL_BaseDiff;
}

static void
fix_compiler_instructions(PInstr *pcpc USES_REGS)
{
  while (pcpc != NULL) {
    PInstr *ncpc = pcpc->nextInst;

    switch(pcpc->op) {
      /* check c_var for functions that point at variables */
    case get_var_op:
    case get_val_op:
    case unify_var_op:
    case unify_last_var_op:
    case unify_val_op:
    case unify_local_op:
    case unify_last_val_op:
    case unify_last_local_op:
    case put_var_op:
    case put_val_op:
    case put_unsafe_op:
    case write_unsafe_op:
    case write_var_op:
    case write_val_op:
    case write_local_op:
    case f_var_op:
    case f_val_op:
    case save_pair_op:
    case save_appl_op:
    case save_b_op:
    case commit_b_op:
    case fetch_args_vv_op:
    case fetch_args_cv_op:
    case fetch_args_vc_op:
      pcpc->rnd1 = GlobalAdjust(pcpc->rnd1);
      break;
    case bccall_op:
      pcpc->rnd1 = GlobalAdjust(pcpc->rnd1);
      pcpc->rnd3 = GlobalAdjust(pcpc->rnd3);
      break;
    case get_float_op:
    case put_float_op:
    case get_longint_op:
    case get_string_op:
    case put_longint_op:
    case put_string_op:
    case unify_float_op:
    case unify_last_float_op:
    case write_float_op:
      /* floats might be in the global */
      pcpc->rnd1 = AdjustAppl(pcpc->rnd1 PASS_REGS);
      break;
      /* hopefully nothing to do */
    case nop_op:
    case ensure_space_op:
    case get_atom_op:
    case put_atom_op:
    case get_num_op:
    case put_num_op:
    case align_float_op:
    case get_bigint_op:
    case put_bigint_op:
    case get_dbterm_op:
    case put_dbterm_op:
    case get_list_op:
    case put_list_op:
    case get_struct_op:
    case put_struct_op:
    case unify_atom_op:
    case unify_last_atom_op:
    case write_atom_op:
    case unify_num_op:
    case unify_last_num_op:
    case write_num_op:
    case unify_longint_op:
    case unify_string_op:
    case unify_last_longint_op:
    case unify_last_string_op:
    case write_longint_op:
    case write_string_op:
    case unify_bigint_op:
    case unify_last_bigint_op:
    case unify_dbterm_op:
    case unify_last_dbterm_op:
    case write_bigint_op:
    case write_dbterm_op:
    case unify_list_op:
    case write_list_op:
    case unify_struct_op:
    case write_struct_op:
    case fail_op:
    case cut_op:
    case cutexit_op:
    case allocate_op:
    case deallocate_op:
    case tryme_op:
    case jump_op:
    case jumpi_op:
    case procceed_op:
    case call_op:
    case execute_op:
    case safe_call_op:
    case label_op:
    case name_op:
    case pop_op:
    case retryme_op:
    case trustme_op:
    case either_op:
    case orelse_op:
    case orlast_op:
    case push_or_op:
    case pushpop_or_op:
    case pop_or_op:
    case patch_b_op:
    case try_op:
    case retry_op:
    case trust_op:
    case try_in_op:
    case jump_v_op:
    case jump_nv_op:
    case cache_arg_op:
    case cache_sub_arg_op:
    case user_switch_op:
    case switch_on_type_op:
    case switch_c_op:
    case if_c_op:
    case switch_f_op:
    case if_f_op:
    case if_not_op:
    case index_dbref_op:
    case index_blob_op:
    case index_long_op:
    case index_string_op:
    case if_nonvar_op:
    case unify_last_list_op:
    case write_last_list_op:
    case unify_last_struct_op:
    case write_last_struct_op:
    case mark_initialized_pvars_op:
    case mark_live_regs_op:
    case enter_profiling_op:
    case retry_profiled_op:
    case count_call_op:
    case count_retry_op:
    case restore_tmps_op:
    case restore_tmps_and_skip_op:
    case enter_lu_op:
    case empty_call_op:
    case blob_op:
    case string_op:
    case fetch_args_vi_op:
    case fetch_args_iv_op:
    case label_ctl_op:
    case f_0_op:
    case native_op:
#ifdef TABLING
    case table_new_answer_op:
    case table_try_single_op:
#endif /* TABLING */
#ifdef YAPOR
    case sync_op:
#endif
#ifdef BEAM
    case run_op:
    case body_op:
    case endgoal_op:
    case try_me_op:
    case retry_me_op:
    case trust_me_op:
    case only_1_clause_op:
    case create_first_box_op:
    case create_box_op:
    case create_last_box_op:
    case remove_box_op:
    case remove_last_box_op:
    case prepare_tries:
    case std_base_op:
    case direct_safe_call_op:
    case commit_op:
    case skip_while_var_op:
    case wait_while_var_op:
    case force_wait_op:
    case write_op:
    case is_op:
    case equal_op:
    case exit_op:
#endif
      break;
    }
    if (ncpc != NULL) {
      ncpc = (PInstr *)GlobalAddrAdjust((ADDR)(pcpc->nextInst));
      pcpc->nextInst = ncpc;
    }
    pcpc = ncpc;
  }
}
static int
do_growheap(int fix_code, UInt in_size, struct intermediates *cip, tr_fr_ptr *old_trp, TokEntry **tksp, VarEntry **vep USES_REGS)
{
  unsigned long size = sizeof(CELL) * K16;
  int shift_factor = (LOCAL_heap_overflows > 8 ? 8 : LOCAL_heap_overflows);
  unsigned long sz =  size << shift_factor;

  if (sz < in_size) {
    sz = in_size;
  }
#ifdef YAPOR
  Yap_Error(RESOURCE_ERROR_HEAP,TermNil,"cannot grow Heap: more than a worker/thread running");
  return FALSE;
#endif
  if (GLOBAL_SizeOfOverflow > sz) {
    if (size < YAP_ALLOC_SIZE)
      size = YAP_ALLOC_SIZE;
    sz = AdjustPageSize(GLOBAL_SizeOfOverflow);
  }
  while(sz >= sizeof(CELL) * K16 && !static_growheap(sz, fix_code, cip, old_trp, tksp, vep PASS_REGS)) {
    size = size/2;
    sz =  size << shift_factor;
    if (sz < in_size) {
      return FALSE;
    }
  }
  /* we must fix an instruction chain */
  if (fix_code) {
    PInstr *pcpc = cip->CodeStart;
    if (pcpc != NULL) {
      cip->CodeStart = pcpc = (PInstr *)GlobalAddrAdjust((ADDR)pcpc);
    }
    fix_compiler_instructions(pcpc PASS_REGS);
    pcpc = cip->BlobsStart;
    if (pcpc != NULL) {
      cip->BlobsStart = pcpc = (PInstr *)GlobalAddrAdjust((ADDR)pcpc);
    }
    fix_compiler_instructions(pcpc PASS_REGS);
    cip->freep = (char *)GlobalAddrAdjust((ADDR)cip->freep);
    cip->label_offset = (Int *)GlobalAddrAdjust((ADDR)cip->label_offset);
  }
  if (sz >= sizeof(CELL) * K16) {
    Yap_get_signal(  YAP_CDOVF_SIGNAL );
    return TRUE;
  }
  /* failed */
  return FALSE;
}

static void
init_new_table(AtomHashEntry *ntb, UInt nsize)
{
  UInt i;

  for (i = 0; i < nsize; i++) {
    INIT_RWLOCK(ntb[i].AERWLock);
    ntb[i].Entry = NIL;
  }
}

static void
cp_atom_table(AtomHashEntry *ntb, UInt nsize)
{
  UInt i;

  for (i = 0; i < AtomHashTableSize; i++) {
    Atom            catom;

    READ_LOCK(HashChain[i].AERWLock);
    catom = HashChain[i].Entry;
    while (catom != NIL) {
      AtomEntry *ap = RepAtom(catom);
      Atom natom;
      CELL hash;

      hash = HashFunction(ap->UStrOfAE) % nsize;
      natom = ap->NextOfAE;
      ap->NextOfAE = ntb[hash].Entry;
      ntb[hash].Entry = catom;
      catom = natom;
    }
    READ_UNLOCK(HashChain[i].AERWLock);
  }
}

static int
growatomtable( USES_REGS1 )
{
  AtomHashEntry *ntb;
  UInt nsize = 3*AtomHashTableSize-1;
  UInt start_growth_time = Yap_cputime(), growth_time;
  int gc_verbose = Yap_is_gc_verbose();
  if (nsize -AtomHashTableSize  > 4*1024*1024)
    nsize =  AtomHashTableSize+4*1024*1024+7919;

  Yap_get_signal(  YAP_CDOVF_SIGNAL );
  while ((ntb = (AtomHashEntry *)Yap_AllocCodeSpace(nsize*sizeof(AtomHashEntry))) == NULL) {
    /* leave for next time */
#if !USE_SYSTEM_MALLOC
    if (!do_growheap(FALSE, nsize*sizeof(AtomHashEntry), NULL, NULL, NULL, NULL))
#endif
      return FALSE;
  }
  LOCAL_atom_table_overflows ++;
  if (gc_verbose) {
#if  defined(YAPOR_THREADS)
    fprintf(stderr, "%% Worker Id %d:\n", worker_id);
#endif
    fprintf(stderr, "%% Atom Table Overflow %d\n", LOCAL_atom_table_overflows );
    fprintf(stderr, "%%    growing the atom table to %ld entries\n", (long int)(nsize));
  }
  YAPEnterCriticalSection();
  init_new_table(ntb, nsize);
  cp_atom_table(ntb, nsize);
  Yap_FreeCodeSpace((char *)HashChain);
  HashChain = ntb;
  AtomHashTableSize = nsize;
  YAPLeaveCriticalSection();
  growth_time = Yap_cputime()-start_growth_time;
  LOCAL_total_atom_table_overflow_time += growth_time;
  if (gc_verbose) {
    fprintf(stderr, "%%   took %g sec\n", (double)growth_time/1000);
    fprintf(stderr, "%% Total of %g sec expanding atom table \n", (double)LOCAL_total_atom_table_overflow_time/1000);
  }
#if USE_SYSTEM_MALLOC
  return TRUE;
#else
  if (HeapTop + sizeof(YAP_SEG_SIZE)  > HeapLim - MinHeapGap) {
    /* make sure there is no heap overflow */
    int res;

    res = do_growheap(FALSE, 0, NULL, NULL, NULL, NULL PASS_REGS);
    return res;
  } else {
    return TRUE;
  }
#endif
}


int
Yap_locked_growheap(bool fix_code, size_t in_size, void *cip)
{
  CACHE_REGS
    int res;
  bool blob_overflow = (NOfBlobs > NOfBlobsMax);

#ifdef THREADS
  LOCK(GLOBAL_BGL);
#endif
  // make sure that we cannot have more than a thread life
  if (Yap_NOfThreads() > 1) {
#ifdef THREADS
    UNLOCK(GLOBAL_BGL);
#endif
    res = FALSE;
    if (NOfAtoms > 2*AtomHashTableSize || blob_overflow) {
      Yap_get_signal( YAP_CDOVF_SIGNAL );
      return TRUE;
    }
  }
  // don't release the MTHREAD lock in case we're running from the C-interface.
  if (NOfAtoms > 2*AtomHashTableSize || blob_overflow) {
    UInt n = NOfAtoms;
    if (GLOBAL_AGcThreshold)
      Yap_atom_gc( PASS_REGS1 );
    /* check if we have a significant improvement from agc */
    if (!blob_overflow &&
	(n > NOfAtoms+ NOfAtoms/10 ||
	 /* +1 = make sure we didn't lose the current atom */
	 NOfAtoms+1 > 2*AtomHashTableSize)) {
      res  = growatomtable( PASS_REGS1 );
    } else {
#ifdef THREADS
      UNLOCK(GLOBAL_BGL);
#endif
      return TRUE;
    }
    LeaveGrowMode(GrowHeapMode);
    if (res) {
#ifdef THREADS
      UNLOCK(GLOBAL_BGL);
#endif
      return res;
    }
  }
#if USE_SYSTEM_MALLOC
  P = Yap_Error(RESOURCE_ERROR_HEAP,TermNil,"malloc failed");
  res = FALSE;
#else
  res=do_growheap(fix_code, in_size, (struct intermediates *)cip, NULL, NULL, NULL PASS_REGS);
#endif
  LeaveGrowMode(GrowHeapMode);
#ifdef THREADS
  UNLOCK(GLOBAL_BGL);
#endif
  return res;
}

int
Yap_growheap(bool fix_code, size_t in_size, void *cip)
{
  int rc;
  rc = Yap_locked_growheap(fix_code, in_size, cip);
  return rc;
}

int
Yap_growheap_in_parser(tr_fr_ptr *old_trp, TokEntry **tksp, VarEntry **vep)
{
  CACHE_REGS
    int res;

  res=do_growheap(FALSE, 0L, NULL, old_trp, tksp, vep PASS_REGS);
  LeaveGrowMode(GrowHeapMode);
  return res;
}

int
Yap_locked_growglobal(CELL **ptr)
{
  CACHE_REGS
    unsigned long sz = sizeof(CELL) * K16;

#if defined(YAPOR_THREADS)
  if (GLOBAL_number_workers != 1) {
    Yap_Error(RESOURCE_ERROR_STACK,TermNil,"cannot grow Global: more than a worker/thread running");
    return(FALSE);
  }
#elif defined(THREADS)
  if (GLOBAL_NOfThreads != 1) {
    Yap_Error(RESOURCE_ERROR_STACK,TermNil,"cannot grow Global: more than a worker/thread running");
    return(FALSE);
  }
#endif
  if ( static_growglobal(sz, ptr, NULL PASS_REGS) == 0)
    return FALSE;
  return TRUE;
}

int
Yap_growglobal(CELL **ptr)
{
  int rc;
  rc = Yap_locked_growglobal(ptr);
  return rc;
}

UInt
Yap_InsertInGlobal(CELL *where, size_t howmuch, CELL **at)
{
  CACHE_REGS
    bool gc_verbose = Yap_is_gc_verbose();
  howmuch = static_growglobal(howmuch, NULL, where PASS_REGS);
  if (gc_verbose) {
#if  defined(YAPOR) || defined(THREADS)
    fprintf(stderr, "%% Worker Id %d:\n", worker_id);
#endif
    fprintf(stderr, "%% Insert %ldB at %p in global\n", howmuch,where);
    show_stacks();
    fprintf(stderr, "%% Growing the stacks " UInt_FORMAT " bytes\n",howmuch);
  }
  if (at) {
    if (LOCAL_GSplit)
      *at = LOCAL_GSplit;
    else
      *at = HR+howmuch/sizeof(CELL);
  }
  return howmuch;
}


int
Yap_locked_growstack(size_t size)
{
  CACHE_REGS
    int res;

  LOCAL_PrologMode |= GrowStackMode;
  res=growstack(size PASS_REGS);
  LeaveGrowMode(GrowStackMode);
  return res;
}

int
Yap_growstack(size_t size)
{
  CACHE_REGS
    int res;

  LOCAL_PrologMode |= GrowStackMode;
  res=growstack(size PASS_REGS);
  LeaveGrowMode(GrowStackMode);
  return res;
}

static int
execute_growstack(size_t esize0, bool from_trail, bool in_parser, tr_fr_ptr *old_trp, TokEntry **tksp, VarEntry **vep USES_REGS)
{
  UInt minimal_request = 0L;
  Int request = esize0;
  Int size = esize0;
  ADDR old_LOCAL_GlobalBase = LOCAL_GlobalBase;

  if (!GLOBAL_AllowGlobalExpansion) {
    LOCAL_ErrorMessage = "Database crashed against stacks";
    return FALSE;
  }
  if (!Yap_ExtendWorkSpace(size)) {
    /* make sure stacks and trail are contiguous */

    LOCAL_ErrorMessage = NULL;
    minimal_request = AdjustPageSize(((CELL)LOCAL_TrailTop-(CELL)LOCAL_GlobalBase)+4*MinHeapGap+request);

    size = Yap_ExtendWorkSpaceThroughHole(minimal_request);
    if (size < 0) {
      LOCAL_ErrorMessage = "Database crashed against stacks";
      return FALSE;
    }
    YAPEnterCriticalSection();
    LOCAL_GDiff = LOCAL_DelayDiff = LOCAL_BaseDiff = size-request;
  } else {
    YAPEnterCriticalSection();
    LOCAL_GDiff = LOCAL_BaseDiff = LOCAL_DelayDiff = LOCAL_GlobalBase-old_LOCAL_GlobalBase;
    LOCAL_GlobalBase=old_LOCAL_GlobalBase;
  }
  LOCAL_XDiff = LOCAL_HDiff = 0;
  LOCAL_GDiff0=LOCAL_GDiff;
#if USE_SYSTEM_MALLOC
  if (from_trail) {
    LOCAL_TrDiff = LOCAL_LDiff = LOCAL_GDiff;
  } else {
    LOCAL_TrDiff = LOCAL_LDiff = size+LOCAL_GDiff;
  }
#else
  if (from_trail) {
    LOCAL_TrDiff = LOCAL_LDiff = size-size0;
  } else {
    LOCAL_TrDiff = LOCAL_LDiff = size;
  }
#endif
  SetHeapRegs(FALSE PASS_REGS);
  if (from_trail) {
    LOCAL_TrailTop += request;
  CurrentTrailTop = (tr_fr_ptr)(LOCAL_TrailTop-MinTrailGap);
  }
  if (LOCAL_LDiff) {
    MoveLocalAndTrail( PASS_REGS1 );
  }
  if (LOCAL_GDiff) {
#if !USE_SYSTEM_MALLOC
    /* That is done by realloc */
    MoveGlobal( PASS_REGS1 );
#endif
    if (in_parser) {
      tr_fr_ptr nTR;

      AdjustScannerStacks(tksp, vep PASS_REGS);
      nTR = TR;
      *old_trp = PtoTRAdjust(*old_trp);
      TR = *old_trp;
      AdjustStacksAndTrail(0, FALSE PASS_REGS);
      TR = nTR;
    } else {
      AdjustStacksAndTrail(0, FALSE PASS_REGS);
    }
    AdjustRegs(MaxTemps PASS_REGS);
  } else if (LOCAL_LDiff) {
    if (in_parser) {
      tr_fr_ptr nTR;

      AdjustScannerStacks(tksp, vep PASS_REGS);
      nTR = TR;
      *old_trp = PtoTRAdjust(*old_trp);
      TR = *old_trp;
      AdjustGrowStack( PASS_REGS1 );
      TR = nTR;
    } else {
      AdjustGrowStack( PASS_REGS1 );
    }
    AdjustRegs(MaxTemps PASS_REGS);
  }
  YAPLeaveCriticalSection();
  if (minimal_request)
    Yap_AllocHole(minimal_request, size);
  return TRUE;
}

/* Used by do_goal() when we're short of stack space */
static int
growstack(size_t size USES_REGS)
{
  UInt start_growth_time, growth_time;
  int gc_verbose;

  /* adjust to a multiple of 256) */
  if (size < YAP_ALLOC_SIZE)
    size = YAP_ALLOC_SIZE;
  size = AdjustPageSize(size);
  LOCAL_ErrorMessage = NULL;
  start_growth_time = Yap_cputime();
  gc_verbose = Yap_is_gc_verbose();
  LOCAL_stack_overflows++;
  if (gc_verbose) {
#if  defined(YAPOR) || defined(THREADS)
    fprintf(stderr, "%% Worker Id %d:\n", worker_id);
#endif
    fprintf(stderr, "%% Stack Overflow %d\n", LOCAL_stack_overflows);
    show_stacks();
    fprintf(stderr, "%% Growing the stacks " UInt_FORMAT " bytes\n", (UInt) size);
  }
  if (!execute_growstack(size, FALSE, FALSE, NULL, NULL, NULL PASS_REGS))
    return FALSE;
  growth_time = Yap_cputime()-start_growth_time;
  LOCAL_total_stack_overflow_time += growth_time;
  if (gc_verbose) {
    fprintf(stderr, "%%   took %g sec\n", (double)growth_time/1000);
    fprintf(stderr, "%% Total of %g sec expanding stacks \n", (double)LOCAL_total_stack_overflow_time/1000);
  }
  return TRUE;
}

/* Used by parser when we're short of stack space */
int
Yap_growstack_in_parser(tr_fr_ptr *old_trp, TokEntry **tksp, VarEntry **vep)
{
  CACHE_REGS
    UInt size;
  UInt start_growth_time, growth_time;
  bool gc_verbose;

  LOCAL_PrologMode |= GrowStackMode;
  /* adjust to a multiple of 256) */
  size = AdjustPageSize((ADDR)LCL0-LOCAL_GlobalBase);
  LOCAL_ErrorMessage = NULL;
  start_growth_time = Yap_cputime();
  gc_verbose = Yap_is_gc_verbose();
  LOCAL_stack_overflows++;
  if (gc_verbose) {
#if  defined(YAPOR) || defined(THREADS)
    fprintf(stderr, "%% Worker Id %d:\n", worker_id);
#endif
    fprintf(stderr, "%% Stack Overflow %d\n", LOCAL_stack_overflows);
    show_stacks();
    fprintf(stderr, "%% Growing the stacks %ld bytes\n", (unsigned long int)size);
  }
  if (!execute_growstack(size, FALSE, TRUE, old_trp, tksp, vep PASS_REGS)) {
    LeaveGrowMode(GrowStackMode);
    return FALSE;
  }
  growth_time = Yap_cputime()-start_growth_time;
  LOCAL_total_stack_overflow_time += growth_time;
  if (gc_verbose) {
    fprintf(stderr, "%%   took %g sec\n", (double)growth_time/1000);
    fprintf(stderr, "%% Total of %g sec expanding stacks \n", (double)LOCAL_total_stack_overflow_time/1000);
  }
  LeaveGrowMode(GrowStackMode);
  return TRUE;
}

static int do_growtrail(size_t esize, bool contiguous_only, bool in_parser, tr_fr_ptr *old_trp, TokEntry **tksp, VarEntry **vep USES_REGS)
{
  UInt start_growth_time = Yap_cputime(), growth_time;
  int gc_verbose = Yap_is_gc_verbose();
  Int size0 =(char*)TR-LOCAL_TrailBase;
  Int size = esize;
  

#if USE_SYSTEM_MALLOC
  if (contiguous_only)
    return FALSE;
#endif
  /* at least 64K for trail */
  if (!size) {
    size = 2*(LOCAL_TrailTop-LOCAL_TrailBase);
  if (size > (LOCAL_TrailTop-LOCAL_TrailBase)+M2*32) {
    size = (size/2 + M2*3);
  }
  } 
  if (size < size0+MinTrailGap) { 
    size += 3*MinTrailGap;
}
	
  /* adjust to a multiple of 256) */
  size = AdjustPageSize(size);
  LOCAL_trail_overflows++;
  if (gc_verbose) {
#if defined(YAPOR) || defined(THREADS)
    fprintf(stderr, "%% Worker Id %d:\n", worker_id);
#endif
    fprintf(stderr, "%% Trail Overflow %d\n", LOCAL_trail_overflows);
    show_stacks();
    fprintf(stderr, "%% growing the trail " UInt_FORMAT " bytes\n", size);
  }
  LOCAL_ErrorMessage = NULL;
  if (!GLOBAL_AllowTrailExpansion) {
    LOCAL_ErrorMessage = "Trail Overflow";
    return FALSE;
  }
#if USE_SYSTEM_MALLOC
  execute_growstack(size, TRUE, in_parser, old_trp, tksp, vep PASS_REGS);
#else
  YAPEnterCriticalSection();
  if (!Yap_ExtendWorkSpace(size)) {
    YAPLeaveCriticalSection();
    LOCAL_ErrorMessage = NULL;
    if (contiguous_only) {
      /* I can't expand in this case */
      LOCAL_trail_overflows--;
      return FALSE;
    }
    execute_growstack(size, TRUE, in_parser, old_trp, tksp, vep PASS_REGS);
  } else {
    if (in_parser) {
      LOCAL_TrDiff = LOCAL_LDiff = LOCAL_GDiff = LOCAL_BaseDiff = LOCAL_DelayDiff = LOCAL_XDiff = LOCAL_HDiff = LOCAL_GDiff0 = 0;
      AdjustScannerStacks(tksp, vep PASS_REGS);
    }
    LOCAL_TrailTop += size;
    CurrentTrailTop = (tr_fr_ptr)(LOCAL_TrailTop-MinTrailGap);
    YAPLeaveCriticalSection();
  }
#endif
  growth_time = Yap_cputime()-start_growth_time;
  LOCAL_total_trail_overflow_time += growth_time;
  if (gc_verbose) {
    fprintf(stderr, "%%  took %g sec\n", (double)growth_time/1000);
    fprintf(stderr, "%% Total of %g sec expanding trail \n", (double)LOCAL_total_trail_overflow_time/1000);
  }
  Yap_get_signal( YAP_TROVF_SIGNAL );
  return TRUE;
}


/* Used by do_goal() when we're short of stack space */
int
Yap_growtrail(size_t size, bool contiguous_only)
{
  int rc;
  CACHE_REGS
    rc = do_growtrail(size, contiguous_only, FALSE, NULL, NULL, NULL PASS_REGS);
  return rc;
}

/* Used by do_goal() when we're short of stack space */
int
Yap_locked_growtrail(size_t size, bool contiguous_only)
{
  CACHE_REGS
    return do_growtrail(size, contiguous_only, FALSE, NULL, NULL, NULL PASS_REGS);
}

int
Yap_growtrail_in_parser(tr_fr_ptr *old_trp, TokEntry **tksp, VarEntry **vep)
{
  CACHE_REGS
    return do_growtrail(0, FALSE, TRUE, old_trp, tksp, vep PASS_REGS);
}

CELL **
Yap_shift_visit(CELL **tovisit, CELL ***tovisit_maxp, CELL ***tovisit_base)
{
  CACHE_REGS
    CELL **tovisit_max = *tovisit_maxp;
  /* relative position of top of stack */
  Int off = (ADDR)tovisit-AuxBase;
  /* how much space the top stack was using */
  Int sz = AuxTop - (ADDR)tovisit_max;
  /* how much space the bottom stack was using */
  Int szlow = (ADDR)tovisit_max-AuxBase;
  /* original size for AuxSpace */
  Int totalsz0 = AuxTop - AuxBase; /* totalsz0 == szlow+sz */
  /* new size for AuxSpace */
  Int totalsz;
  /* how much we grow */
  Int dsz; /* totalsz == szlow+dsz+sz */
  char *newb = Yap_ExpandPreAllocCodeSpace(0, NULL, FALSE);

  if (newb == NULL) {
    Yap_Error(RESOURCE_ERROR_HEAP,TermNil,"cannot allocate temporary space for unification (%p)", tovisit);
    return tovisit;
  }
  /* check new size */
  totalsz  = AuxTop-AuxBase;
  /* how much we grew */
  dsz = totalsz-totalsz0;
  if (dsz == 0) {
    Yap_Error(RESOURCE_ERROR_HEAP,TermNil,"cannot allocate temporary space for unification (%p)", tovisit);
    return tovisit;
  }
  /* copy whole block to end */
  cpcellsd((CELL *)(newb+(dsz+szlow)), (CELL *)(newb+szlow), sz/sizeof(CELL));
  /* base pointer is block start */
  *tovisit_maxp = (CELL **)(newb+szlow);
  /* base pointer is block start */
  if (tovisit_base)
    *tovisit_base = (CELL **)AuxSp;
  /* current top is originall diff + diff size */
  return (CELL **)(newb+(off+dsz));
}

static Int
p_inform_trail_overflows( USES_REGS1 )
{
  Term tn = MkIntTerm(LOCAL_trail_overflows);
  Term tt = MkIntegerTerm(LOCAL_total_trail_overflow_time);

  return(Yap_unify(tn, ARG1) && Yap_unify(tt, ARG2));
}

/* :- grow_heap(Size) */
static Int
p_growheap( USES_REGS1 )
{
  Int             diff;
  Term t1 = Deref(ARG1);

  if (IsVarTerm(t1)) {
    Yap_Error(INSTANTIATION_ERROR, t1, "grow_heap/1");
    return(FALSE);
  } else if (!IsIntTerm(t1)) {
    Yap_Error(TYPE_ERROR_INTEGER, t1, "grow_heap/1");
    return(FALSE);
  }
  diff = IntOfTerm(t1);
  if (diff < 0) {
    Yap_Error(DOMAIN_ERROR_NOT_LESS_THAN_ZERO, t1, "grow_heap/1");
  }
  return(static_growheap(diff, FALSE, NULL, NULL, NULL, NULL PASS_REGS));
}

static Int
p_inform_heap_overflows( USES_REGS1 )
{
  Term tn = MkIntTerm(LOCAL_heap_overflows);
  Term tt = MkIntegerTerm(LOCAL_total_heap_overflow_time);

  return(Yap_unify(tn, ARG1) && Yap_unify(tt, ARG2));
}

#if defined(YAPOR_THREADS)
void
Yap_CopyThreadStacks(int worker_q, int worker_p, bool incremental)
{
  CACHE_REGS
    Int size;

  /* make sure both stacks have same size */
  Int p_size = REMOTE_ThreadHandle(worker_p).ssize+REMOTE_ThreadHandle(worker_p).tsize;
  Int q_size = REMOTE_ThreadHandle(worker_q).ssize+REMOTE_ThreadHandle(worker_q).tsize;
  if (p_size != q_size) {
    UInt start_growth_time, growth_time;
    int gc_verbose;
7    size_t ssiz = REMOTE_ThreadHandle(worker_q).ssize*K1;
    size_t tsiz = REMOTE_ThreadHandle(worker_q).tsize*K1;
    size_t diff = (REMOTE_ThreadHandle(worker_p).ssize-REMOTE_ThreadHandle(worker_q).ssize)*K1;
    char *oldq = (char *)REMOTE_ThreadHandle(worker_q).stack_address, *newq;

    if (!(newq = REMOTE_ThreadHandle(worker_q).stack_address = realloc(REMOTE_ThreadHandle(worker_q).stack_address,p_size*K1))) {
      Yap_Error(RESOURCE_ERROR_STACK,TermNil,"cannot expand slave thread to match master thread");
    }
    start_growth_time = Yap_cputime();
    gc_verbose = Yap_is_gc_verbose();
    LOCAL_stack_overflows++;
    if (gc_verbose) {
#if  defined(YAPOR) || defined(THREADS)
      fprintf(stderr, "%% Worker Id %d:\n", worker_id);
#endif
      fprintf(stderr, "%% Stack Overflow %d\n", LOCAL_stack_overflows);
      show_stacks();
      fprintf(stderr, "%% Growing the stacks %ld bytes\n", diff);
    }
    LOCAL_GDiff = LOCAL_GDiff0 = LOCAL_DelayDiff = LOCAL_BaseDiff = (newq-oldq);
    LOCAL_TrDiff = LOCAL_LDiff = diff + LOCAL_GDiff;
    LOCAL_XDiff = LOCAL_HDiff = 0;
    LOCAL_GSplit = NULL;
    YAPEnterCriticalSection();
    SetHeapRegs(FALSE PASS_REGS);
    {
      choiceptr imageB;

      LOCAL_OldLCL0 = LCL0;
      LCL0 = REMOTE_ThreadHandle(0).current_yaam_regs->LCL0_;
      imageB = Get_GLOBAL_root_cp();
      /* we know B */
      B->cp_tr = TR =
	(tr_fr_ptr)((CELL)(imageB->cp_tr)+((CELL)LOCAL_OldLCL0-(CELL)LCL0));
      LCL0 = LOCAL_OldLCL0;
      B->cp_h = H0;
      B->cp_ap = GETWORK;
      B->cp_or_fr = GLOBAL_root_or_fr;
    }
    YAPLeaveCriticalSection();
    growth_time = Yap_cputime()-start_growth_time;
    LOCAL_total_stack_overflow_time += growth_time;
    if (gc_verbose) {
      fprintf(stderr, "%%   took %g sec\n", (double)growth_time/1000);
      fprintf(stderr, "%% Total of %g sec expanding stacks \n", (double)LOCAL_total_stack_overflow_time/1000);
    }
  }

  REMOTE_ThreadHandle(worker_q).ssize = REMOTE_ThreadHandle(worker_p).ssize;
  REMOTE_ThreadHandle(worker_q).tsize = REMOTE_ThreadHandle(worker_p).tsize;
  /* compute offset indicators */
  LOCAL_GlobalBase = REMOTE_GlobalBase(worker_p);
  LOCAL_LocalBase = REMOTE_LocalBase(worker_p);
  LOCAL_TrailBase = REMOTE_TrailBase(worker_p);
  LOCAL_TrailTop = REMOTE_TrailTop(worker_p);
  CurrentTrailTop = (tr_fr_ptr)(LOCAL_TrailTop-MinTrailGap);
  size = REMOTE_ThreadHandle(worker_q).stack_address-REMOTE_ThreadHandle(worker_p).stack_address;
  LOCAL_TrDiff = LOCAL_LDiff = LOCAL_GDiff = LOCAL_GDiff0 = LOCAL_DelayDiff = LOCAL_BaseDiff = size;
  LOCAL_XDiff = LOCAL_HDiff = 0;
  LOCAL_GSplit = NULL;
  HR = REMOTE_ThreadHandle(worker_p).current_yaam_regs->H_;
  H0 = REMOTE_ThreadHandle(worker_p).current_yaam_regs->H0_;
  B = REMOTE_ThreadHandle(worker_p).current_yaam_regs->B_;
  ENV = REMOTE_ThreadHandle(worker_p).current_yaam_regs->ENV_;
  YENV = REMOTE_ThreadHandle(worker_p).current_yaam_regs->YENV_;
  ASP = REMOTE_ThreadHandle(worker_p).current_yaam_regs->ASP_;
  TR = REMOTE_ThreadHandle(worker_p).current_yaam_regs->TR_;
  if (ASP > CellPtr(B))
    ASP = CellPtr(B);
  LCL0 = REMOTE_ThreadHandle(worker_p).current_yaam_regs->LCL0_;
  Yap_REGS.CUT_C_TOP = REMOTE_ThreadHandle(worker_p).current_yaam_regs->CUT_C_TOP;
  LOCAL_DynamicArrays = NULL;
  LOCAL_StaticArrays = NULL;
  LOCAL_GlobalVariables = NULL;
  SetHeapRegs(TRUE PASS_REGS);
  if (incremental) {
    IncrementalCopyStacksFromWorker( PASS_REGS1 );
    LOCAL_start_global_copy =
      (CELL)PtoGloAdjust((CELL *)LOCAL_start_global_copy);
    LOCAL_end_global_copy =
      (CELL)PtoGloAdjust((CELL *)LOCAL_end_global_copy);
    LOCAL_start_local_copy =
      (CELL)PtoLocAdjust((CELL *)LOCAL_start_local_copy);
    LOCAL_end_local_copy =
      (CELL)PtoLocAdjust((CELL *)LOCAL_end_local_copy);
    LOCAL_start_trail_copy =
      (CELL)PtoTRAdjust((tr_fr_ptr)LOCAL_start_trail_copy);
    LOCAL_end_trail_copy =
      (CELL)PtoTRAdjust((tr_fr_ptr)LOCAL_end_trail_copy);
    AdjustStacksAndTrail(0, STACK_INCREMENTAL_COPYING PASS_REGS);
    RestoreTrail(worker_p PASS_REGS);
    TR = (tr_fr_ptr) LOCAL_end_trail_copy;
  } else {
    CopyLocalAndTrail( PASS_REGS1 );
    MoveGlobal( PASS_REGS1 );
    AdjustStacksAndTrail(0, STACK_COPYING PASS_REGS);
  }
}
#endif

/* :- grow_stack(Size) */
static Int
p_growstack( USES_REGS1 )
{
  Int             diff;
  Term t1 = Deref(ARG1);

  if (IsVarTerm(t1)) {
    Yap_Error(INSTANTIATION_ERROR, t1, "grow_stack/1");
    return(FALSE);
  } else if (!IsIntTerm(t1)) {
    Yap_Error(TYPE_ERROR_INTEGER, t1, "grow_stack/1");
    return(FALSE);
  }
  diff = IntOfTerm(t1);
  if (diff < 0) {
    Yap_Error(DOMAIN_ERROR_NOT_LESS_THAN_ZERO, t1, "grow_stack/1");
  }
  return(growstack(diff PASS_REGS));
}

static Int
p_inform_stack_overflows( USES_REGS1 )
{				/*  */
  Term tn = MkIntTerm(LOCAL_stack_overflows);
  Term tt = MkIntegerTerm(LOCAL_total_stack_overflow_time);

  return(Yap_unify(tn, ARG1) && Yap_unify(tt, ARG2));

}

Int
Yap_total_stack_shift_time(void)
{
  CACHE_REGS
    return(LOCAL_total_heap_overflow_time+
	   LOCAL_total_stack_overflow_time+
	   LOCAL_total_trail_overflow_time);
}

void
Yap_InitGrowPreds(void)
{
  Yap_InitCPred("$grow_heap", 1, p_growheap, SafePredFlag);
  Yap_InitCPred("$grow_stack", 1, p_growstack, SafePredFlag);
  Yap_InitCPred("$inform_trail_overflows", 2, p_inform_trail_overflows, SafePredFlag);
  Yap_InitCPred("$inform_heap_overflows", 2, p_inform_heap_overflows, SafePredFlag);
  Yap_InitCPred("$inform_stack_overflows", 2, p_inform_stack_overflows, SafePredFlag);
  Yap_init_gc();
  Yap_init_agc();
}
