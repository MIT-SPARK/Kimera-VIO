/*
    libranget - optimized 2d range tree
    Copyright (C) 2012 Lauri Kasanen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LIBRANGET
#define LIBRANGET

#include "lrtypes.h"

/* The following block ships the data away and uses an offset to access it.
   It's only used on 64-bit platforms where 64-bit pointers are the majority
   of the memory cost. It saves some 37% of RAM at the cost of slightly slower
   reporting.

   If you value speed over memory use, and are on a 64-bit platform, comment it out.
*/
/*#if defined(__x86_64__) || defined(_LP64)
#define LESSRAM64
#endif*/

#ifdef __GNUC__
#define fetch(a) __builtin_prefetch(a)
#else
#define fetch(a)
#endif

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>

#ifdef LR_VISUALIZE
#include <gvc.h>
#endif

template <class point, class data> class rangetree {

public:
	rangetree(u32 estimatedTotal = 1000, u32 estimatedResult = 100):
		nodepool(NULL), ptypool(NULL),
		mainreserve(estimatedTotal), resultreserve(estimatedResult), init(false) {

		xtmparray.reserve(mainreserve);
		ytmparray.reserve(mainreserve);

#ifdef LESSRAM64
		dataset = NULL;
#endif
	}

	~rangetree() {
		nuke();
		delete [] nodepool;
		free(ptypool);
#ifdef LESSRAM64
		free(dataset);
#endif
	}

	int add(point x, point y, data * const ptr) {

		if (init)
			return 1;

		ptx px;
		pty py;

		px.x = x;
		px.y = py.y = y;
		px.ptr = ptr;
#ifndef LESSRAM64
		py.ptr = ptr;
#else
		py.offset = -1;
#endif

		xtmparray.push_back(px);
		ytmparray.push_back(py);

		return 0;
	}

	void finalize() {

		totalsize = ytmparray.size();

		std::sort(xtmparray.begin(), xtmparray.end());
		std::sort(ytmparray.begin(), ytmparray.end());

#ifdef LESSRAM64
		dataset = (data **) xcalloc(totalsize, sizeof(data *));

		for (u32 i = 0; i < totalsize; i++) {
			dataset[i] = xtmparray[i].ptr;
		}
#endif

		const point maxrange = (xtmparray[totalsize-1].x - xtmparray[0].x) + 1;
		initpools(totalsize, maxrange);

		build();

        xtmparray.clear();

		// Must be last
		init = true;
	}

	u32 count(point xmin, point xmax, point ymin, point ymax) const {

		if (!init)
			return 0;

		// If needed, swap arguments
		if (xmax < xmin)
			pswap(xmax, xmin);
		if (ymax < ymin)
			pswap(ymax, ymin);

		u32 sum = 0;

		const node * list[maxterminals];
		u32 listcur = 0;
		findnodes(&start, xmin, xmax, list, listcur);

		const u32 ncount = listcur;

		for (u32 k = 0; k < ncount; k++) {
			const node * const n = list[k];
			const u32 max = n->ycount;
			if (!max)
				continue;

			const u32 lower = binarynext(n->ypoints, n->ycount, ymin);
			const u32 upper = binarynext(n->ypoints, n->ycount, ymax + 1);

			sum += upper - lower;
		}

		return sum;
	}

	std::vector<data *> *search(point xmin, point xmax, point ymin, point ymax) const {

		if (!init)
			return NULL;

		// If needed, swap arguments
		if (xmax < xmin)
			pswap(xmax, xmin);
		if (ymax < ymin)
			pswap(ymax, ymin);

		std::vector<data *> * const res = new std::vector<data *>;
		res->reserve(resultreserve);

		const node * list[maxterminals];
		u32 listcur = 0;
		findnodes(&start, xmin, xmax, list, listcur);

		const u32 ncount = listcur;

		for (u32 k = 0; k < ncount; k++) {
			const node * const n = list[k];
			const u32 max = n->ycount;
			if (!max)
				continue;

			const u32 lower = binarynext(n->ypoints, n->ycount, ymin);
			const u32 upper = binarynext(n->ypoints, n->ycount, ymax + 1);

			for (u32 i = lower; i < upper; i++) {
#ifdef LESSRAM64
					res->push_back(dataset[n->ypoints[i].offset]);
#else
					res->push_back(n->ypoints[i].ptr);
#endif
			}
		}
		return res;
	}

	// Alternative interface for faster searches, avoiding the mem alloc
	void search(data ** const arr, u32 &arrsize,
			point xmin, point xmax, point ymin, point ymax) const {

		if (!init)
			return;

		// If needed, swap arguments
		if (xmax < xmin)
			pswap(xmax, xmin);
		if (ymax < ymin)
			pswap(ymax, ymin);

		const node * list[maxterminals];
		u32 listcur = 0;
		findnodes(&start, xmin, xmax, list, listcur);

		const u32 ncount = listcur;

		u32 cur = 0;
		const u32 arrmax = arrsize;

		for (u32 k = 0; k < ncount; k++) {
			const node * const n = list[k];
			const u32 max = n->ycount;
			if (!max)
				continue;

			const u32 lower = binarynext(n->ypoints, n->ycount, ymin);
			const u32 upper = binarynext(n->ypoints, n->ycount, ymax + 1);

			for (u32 i = lower; i < upper; i++) {
					if (cur < arrmax) {
#ifdef LESSRAM64
						arr[cur] = dataset[n->ypoints[i].offset];
#else
						arr[cur] = n->ypoints[i].ptr;
#endif
					}

					cur++;
			}
		}

		arrsize = cur;
	}

	static const char *version() {
		return "librangetree 1.3.1";
	}

#ifdef LR_VISUALIZE
	void visualize(const char name[] = "out.png") const {

		if (!init)
			return;

		GVC_t *gvc;
		graph_t *g;
		Agnode_t *nr;

		gvc = gvContext();
		g = agopen((char *)"g", AGDIGRAPH);
		nr = agnode(g, visname(&start));

		visualize(&start, g, nr);

		gvLayout(gvc, g, "dot");
		gvRenderFilename(gvc, g, "png", name);
		gvFreeLayout(gvc, g);

		agclose(g);
		gvFreeContext(gvc);
	}

	void print() {
		print(&start);
	}
#endif

private:
	// Note the order here: pointer after coordinates.
	// If using 16 or 8-bit coords, this wastes space, but
	// it's done like this to improve cache usage (critical word).
	struct ptx {
		point x;
		point y;
		data * ptr;

		inline bool operator < (const ptx &other) const {
			if (x >= other.x)
				return false;
			return true;
		}
	};
	struct pty {
		point y;
#ifdef LESSRAM64
		u32 offset;
#else
		data * ptr;
#endif

		inline bool operator < (const pty &other) const {
			if (y >= other.y)
				return false;
			return true;
		}
	};
	struct node {
		node * left;
		node * right;

		pty *ypoints;
		u32 ycount;

		point min, max;

		node(): left(NULL), right(NULL), ypoints(NULL), ycount(0), min(0), max(0) {}
	};

	// A binary search that returns the index if found, the next index if not
	u32 binarynext(const pty * const arr, const u32 ycount, const point goal) const {

		if (ycount == 0)
			return 0;

		const u32 max = ycount - 1;

		// These have to be signed to avoid overflow. s64 to contain u32.
		s64 t, left = 0, right = max;

		do {
			t = (left + right) / 2;

			if (goal < arr[t].y)
				right = t - 1;
			else
				left = t + 1;

		} while (left <= right && goal != arr[t].y);

		// We might've landed in the middle. Linearly get the earliest.
		if (arr[t].y == goal) {
			while (t && arr[t-1].y == goal)
				t--;
			return t;
		}

		if (arr[t].y < goal)
			return t + 1;

		return t;
	}

	// Same for X-sorted arrays
	u32 binarynextx(const std::vector<ptx> &arr, const point goal) const {

		if (arr.size() == 0)
			return 0;

		const u32 max = arr.size() - 1;

		// These have to be signed to avoid overflow. s64 to contain u32.
		s64 t, left = 0, right = max;

		do {
			t = (left + right) / 2;

			if (goal < arr[t].x)
				right = t - 1;
			else
				left = t + 1;

		} while (left <= right && goal != arr[t].x);

		// We might've landed in the middle. Linearly get the earliest.
		if (arr[t].x == goal) {
			while (t && arr[t-1].x == goal)
				t--;
			return t;
		}

		if (arr[t].x < goal)
			return t + 1;

		return t;
	}

	void build() {
		if (totalsize < 1) {
			return;
		}
#ifndef LESSRAM64
		start.ypoints = (pty *) xcalloc(totalsize, sizeof(pty));
		memcpy(start.ypoints, &ytmparray[0], totalsize * sizeof(pty));
		start.ycount = totalsize;
#endif
        ytmparray.clear();

		const u32 medianidx = totalsize / 2;
		const point median = xtmparray[medianidx].x;

		start.min = xtmparray[0].x;
		start.max = xtmparray[totalsize - 1].x;

		// Ok, divide it between everyone
		start.left = build(start.min, median);

		// If it's really small, it may not have a right branch..
		if (median != start.max)
			start.right = build(median + 1, start.max);

#ifdef LESSRAM64
		mergekids(start.ypoints, start.ycount, start.left, start.right);
#endif
	}

	node *build(const point min, const point max) {

//		if (min > max)
//			abort();

		const u32 lower = binarynextx(xtmparray, min);
		const u32 upper = binarynextx(xtmparray, max + 1);

		// Quick check: if nothing below me, no need to create anything below
		if (lower == upper) {
			return NULL;
		}

		node * const n = newnode();

		n->min = xtmparray[lower].x;
		n->max = xtmparray[upper-1].x;

		// If no kids, create the array; otherwise, recurse
		if (n->min == n->max) {

			const u32 size = upper - lower;

			n->ypoints = newpty(size);

			u32 i;
			for (i = 0; i < size; i++) {
#ifdef LESSRAM64
				n->ypoints[i].offset = lower + i;
#else
				n->ypoints[i].ptr = xtmparray[lower + i].ptr;
#endif
				n->ypoints[i].y = xtmparray[lower + i].y;
			}

			n->ycount = size;

			std::sort(n->ypoints, n->ypoints + size);
		} else {
			const u32 median = (n->min + n->max) / 2;

			n->left = build(n->min, median);
			n->right = build(median + 1, n->max);

			// For faster builds, we merge our kids' arrays into ours
			mergekids(n->ypoints, n->ycount, n->left, n->right);
		}

		return n;
	}

    void nuke() {
		nuke(start.left);
        nuke(start.right);

        if (!isptypooled(start.ypoints))
            delete start.ypoints;
            //free(start.ypoints);
	}

	void nuke(node * const n) {
		if (!n)
            return;

		nuke(n->left);
		nuke(n->right);

        if (!isptypooled(n->ypoints))
            delete n->ypoints;
            //free(n->ypoints);


        if (!isnodepooled(n))
            delete n;
	}

	void findnodes(const node * const n, const point xmin, const point xmax,
			const node * list[], u32 &listcur) const {
		if (!n)
			return;

		// Fast outs
		if (xmin > n->max)
			return;
		if (xmax < n->min)
			return;
		if (!n->ycount)
			return;

		// Prefetch nodes to cache
		fetch(n->left);
		fetch(n->right);

		if (xmin <= n->min && xmax >= n->max) {
			list[listcur] = n;
			listcur++;

//			if (listcur >= maxterminals)
//				abort();

			return;
		}

		findnodes(n->left, xmin, xmax, list, listcur);
		findnodes(n->right, xmin, xmax, list, listcur);
	}

	void mergekids(pty * &arr, u32 &ycount, node * const __restrict__ left,
			node * const __restrict__ right) const {

		u32 l, r;
		const u32 lmax = left ? left->ycount : 0;
		const u32 rmax = right ? right->ycount : 0;

		arr = newpty(lmax + rmax);
		ycount = lmax + rmax;

		u32 cur = 0;

		for (l = 0, r = 0; l < lmax || r < rmax;) {
			// Special cases first: if one array is out
			if (l == lmax) {
				memcpy(&arr[cur], &right->ypoints[r],
					(rmax - r) * sizeof(pty));
				break;
			} else if (r == rmax) {
				memcpy(&arr[cur], &left->ypoints[l],
					(lmax - l) * sizeof(pty));
				break;
			} else {
				if (left->ypoints[l].y <= right->ypoints[r].y) {
					arr[cur] = left->ypoints[l];
					cur++;
					l++;
				} else {
					arr[cur] = right->ypoints[r];
					cur++;
					r++;
				}
			}
		}
	}

#ifdef LR_VISUALIZE
	void visualize(const node * const n, graph_t * const g, Agnode_t * const n1) const {

		Agnode_t *temp;
		Agedge_t *e;

		if (n->left) {
			temp = agnode(g, visname(n->left));
			e = agedge(g, n1, temp);
			agsafeset(e, (char*) "label", (char*) "Left", (char*) "Left");
			visualize(n->left, g, temp);
		}
		if (n->right) {
			temp = agnode(g, visname(n->right));
			e = agedge(g, n1, temp);
			agsafeset(e, (char*) "label", (char*) "Right", (char*) "Right");
			visualize(n->right, g, temp);
		}
	}

	// Yes, this leaks memory. It's a debug option anyhow.
	char *visname(const node * const n) const {
		char *ptr;
		asprintf(&ptr, "%u-%u, %u point%s", n->min, n->max, n->ycount,
			n->ycount > 1 ? "s" : "");
		return ptr;
	}

	void print(const node * const n) {
		printf("Node: max min %u %u, %u points\n", n->max, n->min,
			n->ycount);

		if (n->left) {
			printf("My left:\n");
			print(n->left);
		}
		if (n->right) {
			printf("My right:\n");
			print(n->right);
		}
	}
#endif

	// A memory pool for nodes, to save on the housekeeping overhead,
	// and hopefully gain a bit in cache advantages.
	void initpools(const u32 totalcount, const point maxrange) {

		const u32 amount = u32min(totalcount, maxrange);

		// If we have N points, the likely amount of nodes is 2N - 1.
		nodepoolcount = amount*2;
		nodepoolgiven = 0;

		const u32 log2ed = log2f(nodepoolcount) + 1;

		// Pre-allocate n*log(n) ptys
		ptypoolcount = totalcount * log2ed;
		ptypoolgiven = 0;

		// Given the estimated max, we can calculate the max number
		// of terminal nodes - it's at most 2 per level (proof in the papers).
		maxterminals = log2ed;
		maxterminals *= 2;

		nodepool = new node[nodepoolcount];
		ptypool = (pty *) xcalloc(ptypoolcount, sizeof(pty));
	}

	node * newnode() {
		if (nodepoolgiven < nodepoolcount) {
			node * const out = &nodepool[nodepoolgiven];
			nodepoolgiven++;
			return out;
		} else {
			return new node;
		}
	}

	pty * newpty(const u32 num) const {
		if (ptypoolgiven + num <= ptypoolcount) {
			pty * const out = &ptypool[ptypoolgiven];
			ptypoolgiven += num;
			return out;
		} else {
			return (pty *) xcalloc(num, sizeof(pty));
		}
	}

	bool isnodepooled(const node * const n) const {

		if (!nodepool)
			return false;

		if (n >= &nodepool[0] && n <= &nodepool[nodepoolcount - 1])
			return true;
		return false;
	}

	bool isptypooled(const pty * const p) const {

		if (!ptypool)
			return false;

		if (p >= &ptypool[0] && p <= &ptypool[ptypoolcount - 1])
			return true;
		return false;
	}

	// Misc helpers
	u32 u32min(const u32 a, const u32 b) const {
		if (a < b)
			return a;
		return b;
	}

	void pswap(point & __restrict__ a, point & __restrict__ b) const {
		point tmp = a;
		a = b;
		b = tmp;
	}

	void *xcalloc(const size_t nmemb, const size_t size) const {
		void * const ptr = calloc(nmemb, size);
		if (!ptr)
			abort();
		return ptr;
	}

#ifdef LESSRAM64
	data **dataset;
#endif

	std::vector<ptx> xtmparray;
	std::vector<pty> ytmparray;
	u32 totalsize;

	node start;

	node *nodepool;
	u32 nodepoolcount;
	u32 nodepoolgiven;

	pty *ptypool;
	u32 ptypoolcount;
	mutable u32 ptypoolgiven;

	u32 maxterminals;

	u32 mainreserve, resultreserve;
	bool init;
};

#endif
