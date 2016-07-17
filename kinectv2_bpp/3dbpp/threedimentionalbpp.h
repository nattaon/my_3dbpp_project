#ifndef THREEDIMENTIONALBPP_H
#define THREEDIMENTIONALBPP_H

/* ======================================================================
      3D BIN PACKING, Silvano Martello, David Pisinger, Daniele Vigo
                             1998, 2003, 2006
   ====================================================================== */

/* This code solves the three-dimensional bin-packing problem, which
 * asks for an orthogonal packing of a given set of rectangular-shaped
 * boxes into the minimum number of three-dimensional rectangular bins.
 * Each box j=1,..,n is characterized by a width w_j, height h_j, and
 * depth d_j. An unlimited number of indentical three-dimensional bins,
 * having width W, height H and depth D is available. The boxes have fixed
 * orientation, i.e., they may not be rotated.
 *
 * A description of this code is found in the following papers:
 *
 *   S. Martello, D. Pisinger, D. Vigo, E. den Boef, J. Korst (2003)
 *   "Algorithms for General and Robot-packable Variants of the
 *    Three-Dimensional Bin Packing Problem"
 *   submitted TOMS.
 *
 *   S.Martello, D.Pisinger, D.Vigo (2000)
 *   "The three-dimensional bin packing problem"
 *   Operations Research, 48, 256-267
 *
 * The present code is written in ANSI-C, and has been compiled with
 * the GNU-C compiler using option "-ansi -pedantic" as well as the
 * HP-UX C compiler using option "-Aa" (ansi standard).
 *
 * This file contains the callable routine binpack3d with prototype
 *
 *   void binpack3d(int n, int W, int H, int D,
 *                  int *w, int *h, int *d,
 *                  int *x, int *y, int *z, int *bno,
 *                  int *lb, int *ub,
 *                  int nodelimit, int iterlimit, int timelimit,
 *                  int *nodeused, int *iterused, int *timeused,
 *                  int packingtype);
 *
 * the meaning of the parameters is the following:
 *   n         Size of problem, i.e., number of boxes to be packed.
 *             This value must be smaller than MAXBOXES defined below.
 *   W,H,D     Width, height and depth of every bin.
 *   w,h,d     Integer arrays of length n, where w[j], h[j], d[j]
 *             are the dimensions of box j for j=0,..,n-1.
 *   x,y,z,bno Integer arrays of length n where the solution found
 *             is returned. For each box j=0,..,n-1, the bin number
 *             it is packed into is given by bno[j], and x[j], y[j], z[j]
 *             are the coordinates of it lower-left-backward corner.
 *   lb        Lower bound on the solution value (returned by the procedure).
 *   ub        Objective value of the solution found, i.e., number of bins
 *             used to pack the n boxes. (returned by the procedure).
 *   nodelimit maximum number of decision nodes to be explored in the
 *             main branching tree. If set to zero, the algorithm will
 *             run until an optimal solution is found (or timelimit or
 *             iterlimit is reached). Measured in thousands (see IUNIT).
 *   iterlimit maximum number of iterations in the ONEBIN algorithm
 *             which packs a single bin. If set to zero, the algorithm will
 *             run until an optimal solution is found (or timelimit or
 *             nodelimit is reached). Measured in thousands (see IUNIT).
 *   timelimit Time limit for solving the problem expressed in seconds.
 *             If set to zero, the algorithm will run until an optimal
 *             solution is found; otherwise it terminates after timelimit
 *             seconds with a heuristic solution.
 *   nodeused  returns the number of branch-and-bound nodes investigated,
 *             measured in thousands (see IUNIT).
 *   iterused  returns the number of iterations in ONEBIN algorithm,
 *             measured in thousands (see IUNIT).
 *   timeused  returns the time used in miliseconds
 *   packingtype
 *             Desired packing type. If set to zero, the algorithm will
 *             search for an optimal general packing; if set to one, it
 *             will search for a robot packing.
 *
 * (c) Copyright 1998, 2003, 2005
 *
 *   David Pisinger                        Silvano Martello, Daniele Vigo
 *   DIKU, University of Copenhagen        DEIS, University of Bologna
 *   Universitetsparken 1                  Viale Risorgimento 2
 *   Copenhagen, Denmark                   Bologna, Italy
 *
 * This code can be used free of charge for research and academic purposes
 * only.
 */



#define IUNIT        1000  /* scaling factor of nodes and iterat */
#define MAXBOXES      101  /* max number of boxes (plus one box) */
#define MAXBPP    1000000  /* max numer of iterations in 1-dim bpp */
#define MAXITER      1000  /* max iterations in heuristic onebin_robot */
#define MAXCLOSE       16  /* max level for which try_close is applied */

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include <limits.h>



/* ======================================================================
                   macros
   ====================================================================== */

#define TRUE  1           /* logical variables */
#define FALSE 0

#define WDIM  0           /* rotations of boxes */
#define HDIM  1
#define DDIM  2

#define GENERAL     0     /* packing type */
#define ROBOT       1

#define LEFT   0          /* relative placements */
#define RIGHT  1
#define UNDER  2
#define ABOVE  3
#define FRONT  4
#define BEHIND 5
#define UNDEF  6
#define RELMAX 8

#define STACKDEPTH (MAXBOXES*MAXBOXES*RELMAX)

#define VOL(i)            ((i)->w * (ptype) (i)->h * (i)->d)
#define MINIMUM(i,j)      ((i) < (j) ? (i) : (j))
#define MAXIMUM(i,j)      ((i) > (j) ? (i) : (j))
#define DIF(i,j)          ((int) ((j) - (i) + 1))
#define SWAPINT(a,b)      { register int t; t=*(a);*(a)=*(b);*(b)=t; }
#define SWAP(a,b)         { register box t; t=*(a);*(a)=*(b);*(b)=t; }
#define SWAPI(a,b)        { register itype t; t=(a);(a)=(b);(b)=t; }
#define SWAPP(a,b)        { register point t; t=*(a);*(a)=*(b);*(b)=t; }
#define DF(a,b)           ((r=(a).y-(b).y) != 0 ? r : (a).x-(b).x)


/* ======================================================================
                 type declarations
   ====================================================================== */

typedef short         boolean; /* logical variable      */
typedef short         ntype;   /* number of states,bins */
typedef short         itype;   /* can hold up to W,H,D  */
typedef long          stype;   /* can hold up to W*H*D  */
typedef long          ptype;   /* product multiplication */

/* box record */
typedef struct irec {
  ntype    no;           /* box number                            */
  itype    w;            /* box width  (x-size)                   */
  itype    h;            /* box height (y-size)                   */
  itype    d;            /* box depth  (z-size)                   */
  itype    x;            /* optimal x-position                    */
  itype    y;            /* optimal y-position                    */
  itype    z;            /* optimal z-position                    */
  ntype    bno;          /* bin number                            */
  boolean  k;            /* is the box chosen?                    */
  stype    vol;          /* volume of box                         */
  struct irec *ref;      /* reference to original box (if necessary) */
} box;

/* all problem information */
typedef struct {
  itype    W;            /* x-size of bin                         */
  itype    H;            /* y-size of bin                         */
  itype    D;            /* z-size of bin                         */
  stype    BVOL;         /* volume of a bin                       */
  ntype    n;            /* number of boxes                       */
  boolean  packtype;     /* packing type: GENERAL or ROBOT        */
  box      *fbox;        /* first box in problem                  */
  box      *lbox;        /* last box in problem                   */
  box      *fsol;        /* first box in current solution         */
  box      *lsol;        /* last box in current solution          */
  box      *fopt;        /* first box in optimal solution         */
  box      *lopt;        /* last box in optimal solution          */
  boolean  *closed;      /* for each bin indicator whether closed */
  box      *fclosed;     /* first box in closed bins              */
  box      *lclosed;     /* last box in closed bins               */
  ntype    noc;          /* number of closed bins                 */
  itype    mindim;       /* currently smallest box length         */
  itype    maxdim;       /* currently largest box length          */
  stype    maxfill;      /* the best filling found                */
  int      mcut;         /* how many siblings at each node in b&b */

  /* different bounds */
  ntype    bound0;       /* Bound L_0 at root node                */
  ntype    bound1;       /* Bound L_1 at root node                */
  ntype    bound2;       /* Bound L_2 at root node                */
  ntype    lb;           /* best of the above                     */
  ntype    z;            /* currently best solution               */

  /* controle of 3d filler */
  int      maxiter;      /* max iterations in onebin_robot        */
  int      miss;         /* number boxes not packed in onebin_robot */

  /* debugging and controle information */
  int      nodes;        /* nodes in branch-and-bound             */
  int      iterat;       /* iterations in onebin_decision         */
  int      subnodes;     /* nodes in branch-and-bound             */
  int      subiterat;    /* iterations in onebin_decision         */
  int      exfill;       /* number of calls to onebin_decision    */
  int      iter3d;       /* iterations in onebin_robot or general */
  int      zlayer;       /* heuristic solution layer              */
  int      zmcut;        /* heuristic solution mcut               */
  double   exacttopo;    /* number of topological sorts           */
  double   exacttopn;    /* number of topological sorts           */
  int      exactcall;    /* number of calls to exact              */
  int      exactn;       /* largest problem for exact             */
  double   genertime;    /* time used in onebin_general           */
  double   robottime;    /* time used in onebin_robot             */
  double   time;         /* computing time                        */
  double   lhtime;       /* layer heuristic computing time        */
  double   mhtime;       /* mcut heuristic computing time         */
  int      didpush;      /* did the lower bound push up bound     */
  int      maxclose;     /* max number of closed bins at any time */
  int      nodelimit;    /* maximum number of nodes in main tree  */
  int      iterlimit;    /* maximum number of iterations in ONEBIN*/
  int      timelimit;    /* maximum amount of time to be used     */
} allinfo;

/* structure for greedy algorithm */
typedef struct {
  int      lno;          /* layer number                          */
  int      d;            /* depth of layer                        */
  int      bno;          /* bin no assigned to layer              */
  int      z;            /* z level of layer within bin           */
  int      b;            /* temporary bin number                  */
} heurpair;

/* structure for extreme points in a single bin */
typedef struct {
  itype    x;            /* x-coordinate                          */
  itype    y;            /* y-coordinate                          */
  itype    z;            /* z-coordinate                          */
} point;

/* structure for a domain pair in constraint programming */
typedef struct {
  int i;                 /* index of box i                        */
  int j;                 /* index of box j                        */
  int relation;          /* relation between the two boxes        */
  boolean domain;        /* domain of the two boxes               */
} domainpair;


/* set of domains */
typedef char domset[RELMAX];
typedef domset domline[MAXBOXES];

/* pointer to comparison function */
typedef int (*funcptr) (const void *, const void *);


/* ======================================================================
                  global variables
   ====================================================================== */

/* boolean variable to indicate time-out situation */
boolean stopped;

/* counter used to ensure that 1D BPP at most performs MAXBPP iterations */
int bpiterat;

/* boolean variables to indicate when 1D packing algorithm should terminate */
boolean feasible, terminate;

/* stack of domain pairs */
domainpair domstack[STACKDEPTH];
domainpair *dompos, *domend;

/* domain of each box */
domline domain[MAXBOXES];

/* current relation between two boxes */
char relation[MAXBOXES][MAXBOXES];

/* debug variable to see level in recursive packing algorithm */
int bblevel;


class ThreeDimentionalBPP
{
public:
    ThreeDimentionalBPP();
    void binpack3d(int n, int W, int H, int D,
                   int *w, int *h, int *d,
                   int *x, int *y, int *z, int *bno,
                   int *lb, int *ub,
                   int nodelimit, int iterlimit, int timelimit,
                   int *nodeused, int *iterused, int *timeused,
                   int packingtype);
    void printboxes_array(int size, int* w, int* h, int* d, int* bno, int* x, int* y, int* z);


private:
    void error(char *str, ...);
    void timer(double *time);
    void check_timelimit(long max);
    void check_nodelimit(long nodes, long max);
    void check_iterlimit(long iterations, long max);

    int dcomp(box *a, box *b) ;
    int hcomp(box *a, box *b) ;
    int vcomp(box *a, box *b);
    int xcomp(heurpair *a, heurpair *b);
    int lcomp(heurpair *a, heurpair *b);

    void *palloc(long sz, long no);
    void pfree(void *p);

    void checksol(allinfo *a, box *f, box *l);
    void savesol(allinfo *a, box *f, box *l, ntype z);
    void isortincr(int *f, int *l);
	void isortdecr(int *f, int *l);

    void psortdecr(point *f, point *l);

    int bound_zero(allinfo *a, box *f, box *l);
    void rotate_solution(allinfo *a, box *f, box *l);
    void rotate_problem(allinfo *a, box *f, box *l);
    void choose_boxes(allinfo *a, box *f, box *l, int W2, int D2,
              box *fbox , box **lbox);
    void find_plist(box *fbox, box *lbox, itype M, int dim, int *pl);
    int bound_one_x(allinfo *a, box *f, box *l);
    int bound_one(allinfo *a, box *f, box *l);

    int bound_two_x(allinfo *a, box *f, box *l);
    int bound_two(allinfo *a, box *f, box *l);

    void onelayer(allinfo *a, box *f, box *m, box *l, int d);
    box *countarea(box *f, box *l, stype barea);
    box *remboxes(box *f, box *l, itype *depth);

    void assignboxes(heurpair *t, heurpair *u, ntype maxbin, box *f, box *l);
    void onedim_binpack(heurpair *i, heurpair *f, heurpair *l,
                int *b, int bno, itype *z);
    void dfirst_heuristic(allinfo *a);
    void dfirst3_heuristic(allinfo *a);
    void modifyandpush(int i, int j, int rel, boolean dom);
    void popdomains(domainpair *pos);

    boolean findcoordinates(allinfo *a, int n, box *f);
    void checkdomain(allinfo *a, int i, int j, int n, box *f, int value);
    boolean reducedomain(allinfo *a, int n, box *f);
    void recpack(allinfo *a, int i, int j, int n, box *f, int rel);

    boolean general_pack(allinfo *a, box *f, box *l);
    boolean onebin_general(allinfo *a, box *f, box *l, boolean fast);

    void envelope(point *f, point *l, point *s1, point **sm,
             itype W, itype H, itype D, itype RW, itype RH, itype cz,
                 point **ll, int *nz, stype *area);
    void checkdom(point *s1, point *sl, point *sm);
    point *removedom(point *s1, point *sl);
    void initboxes(box *f, box *l, point *fc, point **lc,
                   int *minw, int *minh, int *mind);
    stype findplaces(allinfo *a, box *f, box *l,
                     point *s1, point **sm, stype fill);

    void branch(allinfo *a, box *f, box *l, int miss, stype fill);
    boolean onebin_robot(allinfo *a, box *f, box *l, boolean fast);
    void mcut_heuristic(allinfo *a);
    void mcut3_heuristic(allinfo *a);

    boolean fits2(box *i, box *j, itype W, itype H, itype D);
    boolean fits2p(box *i, box *j, itype W, itype H, itype D);
    boolean fits3(box *i, box *j, box *k, itype W, itype H, itype D, int packtype);
    boolean fitsm(allinfo *a, box *t, box *k, boolean fast);

    boolean onebin_decision(allinfo *a, box *j, int bno);
    boolean onebin_heuristic(allinfo *a, box *f, box *l);
    boolean try_close(allinfo *a, box **curr, ntype bno,
                     box *oldf, box **oldl, box **oldlc, ntype *oldnoc,
                     boolean *oldclosed, int level);

    void free_close(allinfo *a, ntype bno,
                    box *oldf, box *oldl, box *oldlc, ntype oldnoc,
                    boolean *oldclosed);
    void rec_binpack(allinfo *a, box *i, int bno, ntype lb, int level);

    void clearboxes(allinfo *a);
    void copyboxes(allinfo *a, int *w, int *h, int *d, int W, int H, int D);
    void returnboxes(allinfo *a, int *x, int *y, int *z, int *bno);


};

#endif // THREEDIMENTIONALBPP_H
