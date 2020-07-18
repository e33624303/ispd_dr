#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <string>
#include <list>
#include <time.h>
#include <unordered_map>
#include <bitset>

#define UndefineValue -1
using namespace std;

template <class V> class FibonacciHeap;
template <class V>
struct node
{
  private:
	node<V> *prev;
	node<V> *next;
	node<V> *child;
	node<V> *parent;
	V value;
	int degree;
	bool marked;

  public:
	friend class FibonacciHeap<V>;
	node<V> *getPrev() { return prev; }
	node<V> *getNext() { return next; }
	node<V> *getChild() { return child; }
	node<V> *getParent() { return parent; }
	V getValue() { return value; }
	bool isMarked() { return marked; }

	bool hasChildren() { return child; }
	bool hasParent() { return parent; }
};

template <class V>
class FibonacciHeap
{
  protected:
	node<V> *heap;

  public:
	FibonacciHeap()
	{
		heap = _empty();
	}
	virtual ~FibonacciHeap()
	{
		if (heap)
		{
			_deleteAll(heap);
		}
	}
	node<V> *insert(V value)
	{
		node<V> *ret = _singleton(value);
		heap = _merge(heap, ret);
		return ret;
	}
	void merge(FibonacciHeap &other)
	{
		heap = _merge(heap, other.heap);
		other.heap = _empty();
	}

	bool isEmpty()
	{
		return heap == NULL;
	}

	V getMinimum()
	{
		return heap->value;
	}

	V removeMinimum()
	{
		node<V> *old = heap;
		heap = _removeMinimum(heap);
		V ret = old->value;
		delete old;
		return ret;
	}

	void decreaseKey(node<V> *n, V value)
	{
		heap = _decreaseKey(heap, n, value);
	}

	node<V> *find(V value)
	{
		return _find(heap, value);
	}

  private:
	node<V> *_empty()
	{
		return NULL;
	}

	node<V> *_singleton(V value)
	{
		node<V> *n = new node<V>;
		n->value = value;
		n->prev = n->next = n;
		n->degree = 0;
		n->marked = false;
		n->child = NULL;
		n->parent = NULL;
		return n;
	}

	node<V> *_merge(node<V> *a, node<V> *b)
	{
		if (a == NULL)
			return b;
		if (b == NULL)
			return a;
		if (a->value > b->value)
		{
			node<V> *temp = a;
			a = b;
			b = temp;
		}
		node<V> *an = a->next;
		node<V> *bp = b->prev;
		a->next = b;
		b->prev = a;
		an->prev = bp;
		bp->next = an;
		return a;
	}

	void _deleteAll(node<V> *n)
	{
		if (n != NULL)
		{
			node<V> *c = n;
			do
			{
				node<V> *d = c;
				c = c->next;
				_deleteAll(d->child);
				delete d;
			} while (c != n);
		}
	}

	void _addChild(node<V> *parent, node<V> *child)
	{
		child->prev = child->next = child;
		child->parent = parent;
		parent->degree++;
		parent->child = _merge(parent->child, child);
	}

	void _unMarkAndUnParentAll(node<V> *n)
	{
		if (n == NULL)
			return;
		node<V> *c = n;
		do
		{
			c->marked = false;
			c->parent = NULL;
			c = c->next;
		} while (c != n);
	}

	node<V> *_removeMinimum(node<V> *n)
	{
		_unMarkAndUnParentAll(n->child);
		if (n->next == n)
		{
			n = n->child;
		}
		else
		{
			n->next->prev = n->prev;
			n->prev->next = n->next;
			n = _merge(n->next, n->child);
		}
		if (n == NULL)
			return n;
		node<V> *trees[64] = {NULL};

		while (true)
		{
			if (trees[n->degree] != NULL)
			{
				node<V> *t = trees[n->degree];
				if (t == n)
					break;
				trees[n->degree] = NULL;
				if (n->value < t->value)
				{
					t->prev->next = t->next;
					t->next->prev = t->prev;
					_addChild(n, t);
				}
				else
				{
					t->prev->next = t->next;
					t->next->prev = t->prev;
					if (n->next == n)
					{
						t->next = t->prev = t;
						_addChild(t, n);
						n = t;
					}
					else
					{
						n->prev->next = t;
						n->next->prev = t;
						t->next = n->next;
						t->prev = n->prev;
						_addChild(t, n);
						n = t;
					}
				}
				continue;
			}
			else
			{
				trees[n->degree] = n;
			}
			n = n->next;
		}
		node<V> *min = n;
		do
		{
			if (n->value < min->value)
				min = n;
			n = n->next;
		} while (n != n);
		return min;
	}

	node<V> *_cut(node<V> *heap, node<V> *n)
	{
		if (n->next == n)
		{
			n->parent->child = NULL;
		}
		else
		{
			n->next->prev = n->prev;
			n->prev->next = n->next;
			n->parent->child = n->next;
		}
		n->next = n->prev = n;
		n->marked = false;
		return _merge(heap, n);
	}

	node<V> *_decreaseKey(node<V> *heap, node<V> *n, V value)
	{
		if (n->value < value)
			return heap;
		n->value = value;
		node<V> *parent = n->parent;
		if (parent != NULL && n->value < n->parent->value)
		{
			heap = _cut(heap, n);
			n->parent = NULL;
			while (parent != NULL && parent->marked)
			{
				heap = _cut(heap, parent);
				n = parent;
				parent = n->parent;
				n->parent = NULL;
			}
			if (parent != NULL && parent->parent != NULL)
				parent->marked = true;
		}
		return heap;
	}

	node<V> *_find(node<V> *heap, V value)
	{
		node<V> *n = heap;
		if (n == NULL)
			return NULL;
		do
		{
			if (n->value == value)
				return n;
			node<V> *ret = _find(n->child, value);
			if (ret)
				return ret;
			n = n->next;
		} while (n != heap);
		return NULL;
	}
};

template <class V> class pheap;
template <class V>
struct pheap_el
{
	struct pheap_el<V> *lchild;
	struct pheap_el<V> *left;
	struct pheap_el<V> *right;
	V * data;
	friend class pheap<V>;
};

template <class V>
class pheap
{

  public:
	pheap_el<V> *root;
	int size;
	void initial()
	{
		root = NULL;
		size = 0;
	}
	void clear()
	{
		root = NULL;
		size = 0;
		//numStore=0;
	}
	void memory_clear(){
		while(root != NULL){
			free(extract());
		}
	}
	void insert(pheap_el<V> *el)
	{
		//printf("Insert start  size=%d\n", size);
		size++;
		el->lchild = el->left = el->right = NULL;
		if (root == NULL)
			root = el;
		else
		{
			//store[numStore]=el;
			//numStore++;
			root = cmpAndlink(root, el);
		}
		//printf("insert COST=%f.2  size=%d\n", ((GridCell *)root->data)->totalCost, size);
	}
	pheap_el<V> *extract()
	{
		//printf("ETRACT start  size=%d\n", size);
		size--;
		if (size < 0)
		{
			printf("ERROR %s %d\n", __FILE__, __LINE__);
			exit(1);
		}

		//multiPassPairing(store, numStore);
		/*
	twoPassPairing(store, numStore);
	if(numStore>0)
	{
		numStore=0;
		root=cmpAndlink(root, store[0]);
	}
	*/
		pheap_el<V> *oldRoot = root;
		if (root->lchild == NULL)
			root = NULL;
		else
			root = combineSiblings(root->lchild);

		oldRoot->lchild = oldRoot->left = oldRoot->right = NULL;
		//printf("ETRACT COST=%f.2  size=%d\n", ((GridCell *)oldRoot->data)->totalCost, size);
		return oldRoot;
	}
	void decreaseKey(pheap_el<V> *el, float newValue)
	{
		//printf("Decrase start  size=%d\n", size);
		if (newValue > (el->data)->cost)
		{
			printf("ERROR %s %d\n", __FILE__, __LINE__);
			exit(1);
		}
		(el->data)->cost = newValue;
		if (el != root)
		{
			//if(el->left!=NULL)
			{
				//	printf("stop 1\n"); fflush(stdout);
				if (el->right != NULL)
					el->right->left = el->left;
				//	printf("stop 2\n"); fflush(stdout);
				if (el->left->lchild == el)
					el->left->lchild = el->right;
				else
					el->left->right = el->right;
				//	printf("stop 3\n"); fflush(stdout);
				el->left = el->right = NULL;

				//store[numStore]=el;
				//numStore++;
				root = cmpAndlink(root, el);
			}

			//	printf("stop 4\n"); fflush(stdout);
		}
	}

  private:
	
	//vector<pheap_el *>	store;
	//int numStore;
	inline pheap_el<V> *cmpAndlink(pheap_el<V> *e1, pheap_el<V> *e2)
	{
		/*
	if(e1->left!=NULL || e1->right!=NULL)
	{
		printf("ERROR %s %d\n", __FILE__, __LINE__);
		exit(1);
	}

	if(e2->left!=NULL || e2->right!=NULL)
     {
          printf("ERROR %s %d\n", __FILE__, __LINE__);
          exit(1);
     }
	*/
		if ((e1->data)->cost < (e2->data)->cost)
		{
			e2->left = e1;
			e2->right = e1->lchild;
			if (e2->right != NULL)
				e2->right->left = e2;
			e1->lchild = e2;
			return e1;
		}
		else
		{
			e1->left = e2;
			e1->right = e2->lchild;
			if (e1->right != NULL)
				e1->right->left = e1;
			e2->lchild = e1;
			return e2;
		}
	}

	inline pheap_el<V> *combineSiblings(pheap_el<V> *left)
	{
		vector<pheap_el<V> *> tmp(size);
		int num = 0;
		pheap_el<V> *tmpEl1;
		pheap_el<V> *tmpEl2;
		//printf("start\n"); fflush(stdout);
		while (left != NULL)
		{
			if (left->right != NULL)
			{
				//printf("merge\n");
				tmpEl1 = left;
				tmpEl2 = left->right;
				left = left->right->right;
				tmpEl1->left = tmpEl1->right = tmpEl2->left = tmpEl2->right = NULL;
				tmp[num] = cmpAndlink(tmpEl1, tmpEl2);
			}
			else
			{
				//printf("end\n");
				left->left = NULL;
				tmp[num] = left;
				left = NULL;
			}
			num++;
		}
		//printf("num=%d\n", num); fflush(stdout);
		for (num = num - 1; num >= 1; num--)
			tmp[num - 1] = cmpAndlink(tmp[num - 1], tmp[num]);
		//printf("end\n", num); fflush(stdout);

		return tmp[0];
		/*
	//useful
	pheap_el *tmpEl;
	int num=0;
	for(tmpEl=left; tmpEl!=NULL; num++)
	{
		tmp[num]=tmpEl;
		tmpEl=tmpEl->right;
		tmp[num]->left = tmp[num]->right =NULL;
	}
	*/

		/*
	printf("combine stage 1 num=%d\n", num); fflush(stdout);
	if(num>4)
	{
	 	printf("ERROR num>4 %s %d\n", __FILE__, __LINE__);
		getchar();
          //exit(1);
	}
	*/

		//if(num<8)
		/*
	if(num>16)
	{
		multiPassPairing(tmp, num);
	//for(index=num-2; index>=0; index--)
	//	tmp[index] = cmpAndlink(tmp[index], tmp[index+1]);
	
	}
	else*/
		/*
	//useful
	{
	//twoPassPairing(tmp, num);

	int index;
	
	for(index=0; index+1<num; index+=2)
		 tmp[index] = cmpAndlink(tmp[index], tmp[index+1]);
	//printf("stage 2 index=%d  num=%d\n", index, num); fflush(stdout);
	int j=index-2;
	//if(j==num-3 && num>2)
	if( num>2 && (num&1)==1)
		tmp[j] = cmpAndlink(tmp[j], tmp[j+2]);

	//printf("stage 3\n"); fflush(stdout);
	for(;j>=2; j-=2)
		 tmp[j-2] = cmpAndlink(tmp[j-2], tmp[j]);
	
	}

	//printf("combine stage 4\n"); fflush(stdout);
	return tmp[0];
	*/
	}

	inline void twoPassPairing(vector<pheap_el<V> *> &forest, int num)
	{
		//for(int index=num-2; index>=0; index--)
		//	forest[index] = cmpAndlink(forest[index], forest[index+1]);

		int index;
		//printf("num=%d\n", num);
		for (index = 0; index + 1 < num; index += 2)
		{
			//printf("index=%d offset=%d\n", index, 1);
			forest[index] = cmpAndlink(forest[index], forest[index + 1]);
		}
		int j = index - 2;
		if (num > 2 && (num & 1) == 1)
		{
			//printf("index=%d offset=%d\n", j, 2);
			forest[j] = cmpAndlink(forest[j], forest[j + 2]);
		}
		for (; j >= 2; j -= 2)
		{
			//printf("index=%d offset=%d\n", j, -2);
			forest[j - 2] = cmpAndlink(forest[j - 2], forest[j]);
		}
	}

	inline void multiPassPairing(vector<pheap_el<V> *> &forest, int num)
	{
		//printf("multiPass %d\n", num); fflush(stdout);
		//getchar();
		int offset = 1;
		int gap = 2;
		while (num > 1)
		{
			//if(num&1==1)
			//	forest[num-2] = cmpAndlink(forest[num-2], forest[num-1]);
			//printf("====================num= %d  =======================\n", num); fflush(stdout);
			int index, i;

			//for(index=0; index+offset<num; index+=gap)
			for (i = 0; i < (num / 2); i++)
			{
				index = i * gap;
				if (forest[index]->left != NULL || forest[index]->right != NULL)
				{
					printf("ERROR %s %d\n", __FILE__, __LINE__);
					exit(1);
				}
				//printf("index=%d offset=%d\n", index, offset);
				forest[index] = cmpAndlink(forest[index], forest[index + offset]);
			}
			if (num & 1 == 1)
			{
				//printf("index=%d \n", index);
				forest[0] = cmpAndlink(forest[0], forest[i * gap]);
			}
			num = num / 2;
			gap = gap * 2;
			offset = offset * 2;
		}
		//printf("multiPass end\n"); fflush(stdout);
	}
};



namespace Parser{

	typedef pair <float, float> FloatPair;
	typedef pair <int, int > IntPair;
	
	
	class INT_EOL_Rule{
	public:
		int SPACING;
		int ENDOFLINE;
		int WITHIN;
		void clear(){
			this->SPACING = UndefineValue;
			this->ENDOFLINE = UndefineValue;
			this->WITHIN = UndefineValue;
		}
	};

	class ViaTypeNote{
	public:
	  pair<int8_t, int8_t> RaisingVia;
	  pair<int8_t, int8_t> FallingVia;
	  void clear()
	  {
		  this->RaisingVia = make_pair(UndefineValue, UndefineValue);
		  this->FallingVia = make_pair(UndefineValue, UndefineValue);
		}
	};

	class EOL_Rule{
	public:
		float SPACING;
		float ENDOFLINE;
		float WITHIN;
		void clear(){
			this->SPACING = UndefineValue;
			this->ENDOFLINE = UndefineValue;
			this->WITHIN = UndefineValue;
		}
	};

	struct I_Rect
	{
	  public:
		IntPair LB;
		IntPair RT;
		// for port
		int Layer;
		I_Rect(int _lbx, int _lby, int _rtx, int _rty, int _layer)
		{
			this->LB = make_pair(_lbx, _lby);
			this->RT = make_pair(_rtx, _rty);
			this->Layer = _layer;
		}
		I_Rect(){
			this->LB = make_pair(UndefineValue, UndefineValue);
			this->RT = make_pair(UndefineValue, UndefineValue);
			this->Layer = UndefineValue;
		}

		void clear()
		{
			this->LB = make_pair(UndefineValue, UndefineValue);
			this->RT = make_pair(UndefineValue, UndefineValue);
		}

		bool SameRect (const I_Rect &rect){
			return (this->LB.first == rect.LB.first) & (this->LB.second == rect.LB.second) & (this->RT.second == rect.RT.second) & (this->RT.first == rect.RT.first);
		}
		
	};

	class Rect{
	public:
		FloatPair LB;
		FloatPair RT;
		// for port
		string Layer;
		void clear(){
			this->LB = make_pair(UndefineValue, UndefineValue);
			this->RT = make_pair(UndefineValue, UndefineValue);
		}
	};

	class SpacingTable{
	public:
		float PARALLELRUNLENGTH;
		vector < FloatPair > WIDTH;
		void clear(){
			this->PARALLELRUNLENGTH = UndefineValue;
			WIDTH.clear();
		}
	};

	class PORT{
	public:
		string LAYER;
		vector <Rect> rect_list;
		void clear(){
			LAYER.clear();
			rect_list.clear();
		}
	};

	class PIN{
	public:
		string Name;
		string USE;
		string DIRECTION;
		// For Macro Pin
		string Shape;
		PORT port;
		// For Routing Pin
		string Layer;
		Rect LayerBound;
		string Net_belong;
		IntPair Place_Index;
		string Orient;
		void clear(){
			DIRECTION.clear();
			port.clear();
			Name.clear();
			USE.clear();
			Shape.clear();
			Layer.clear();
			LayerBound.clear();
			Net_belong.clear();
			Place_Index = make_pair(UndefineValue, UndefineValue);
			Orient.clear();
		}
	};

	class Metal{
	public:
		string Name;
		bool DIRECTION;
		float MINWIDTH;
		float AREA;
		float WIDTH;
		float SPACING;
		EOL_Rule EOL;
		FloatPair PITCH;
		SpacingTable spacing_table;
		void clear(){
			Name.clear();
			DIRECTION = false;
			MINWIDTH = UndefineValue;
			AREA = UndefineValue;
			WIDTH = UndefineValue;
			SPACING = UndefineValue;
			EOL.clear();
			spacing_table.clear();
			PITCH = make_pair(UndefineValue, UndefineValue);
		}
	};

	class ViaLayer{
	public:
		string Name;
		string TYPE;
		float CUT_SPACING;
		void clear(){
			Name.clear();
			TYPE.clear();
			CUT_SPACING = UndefineValue;
		}
	};

	class LayerInfo{
	public:
		bool Type;
		Metal metalLayer; // true
		ViaLayer viaLayer; // false

		void clear(){
			Type = false;
			metalLayer.clear();
			viaLayer.clear();
		}
	};

	class Via{
	public:
		bool HasDefault;
		string Name;
		// Bot Layer
		string BotLayerName;
		Rect BotLayerRect;
		int BotLayer;
		I_Rect  BotLayerIRect;
		// Via Layer
		string ViaLayerName;
		Rect ViaLayerRect;
		int ViaLayer;
		I_Rect  ViaLayerIRect;
		// Top Layer
		string TopLayerName;
		Rect TopLayerRect;
		int TopLayer;
		I_Rect  TopLayerIRect;
		void clear(){
			HasDefault = false;
			Name.clear();
			BotLayerName.clear();
			BotLayerRect.clear();
			ViaLayerName.clear();
			ViaLayerRect.clear();
			TopLayerName.clear();
			TopLayerRect.clear();
		}
	};

	class MACRO{
	public:
		string Name;
		string CLASS;
		vector <PIN> Pin_list;
		FloatPair SIZE_BY;
		FloatPair ORIGIN;
		void clear(){
			Name.clear();
			CLASS.clear();
			Pin_list.clear();
			SIZE_BY = make_pair(UndefineValue, UndefineValue);
			ORIGIN = make_pair(UndefineValue, UndefineValue);
		}
	};

	class Component{
	public:
		string Name;
		string ID;
		IntPair index;
		string Dir;
	};

	class Row{
	public:
		string Name;
		string Macro;
		IntPair start_index;
		string Dir;
		IntPair Do_By;
		IntPair Step;
	};

	class Track{
	public:
		string Dir;
		int start_index;
		int Do;
		int Step;
		string Layer;
		int i_layer;
	};

	class RipUpNode
	{
	  public:
		int x;
		int y;
		int z;

		RipUpNode(){
			x = UndefineValue;
			y = UndefineValue;
			z = UndefineValue;
		}

		RipUpNode(int _x, int _y, int _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}
	};

	class Component_Port_Name{
	public:
		string Component_Name;
		string Pin_Name;
	};

	class Net_Name{
	public:
		string Net_name;
		vector <Component_Port_Name> CPN_list;
		void clear(){
			Net_name.clear();
			CPN_list.clear();
		}
	};

	class ISPD_Routing_Port_Rect{
	public:
		int Layer;
		vector <Rect> Rect_list;
	};

	class IRoute
	{
	  public:
		IntPair LB;
		IntPair RT;
		int Layer;
		int Net_index;
	};

	class RipUpSegment
	{
	public:
		I_Rect SpacingMetal;
		I_Rect EOL_SpacingMetal;
		int Layer;
		IntPair TreeID; // spacing, eol_spacing

		int ALL_shape_index;
		
		bool IsVia;
		int via_x, via_y, via_z;
		vector<RipUpNode> Via_obstacle_node;
		vector<RipUpNode> Via_EOL_obstacle_node;

		RipUpSegment()
		{
			Layer = UndefineValue;
			TreeID = make_pair(UndefineValue, UndefineValue);
			ALL_shape_index = UndefineValue;
			IsVia = false;
			via_x = UndefineValue;
			via_y = UndefineValue;
			via_z = UndefineValue;
		}
	};

	class wire_path{
	public:
		int path_type;  // 0 : only wire , 1 : has wire + via , 2 : only src pin + via , 3 : rect patch
		int Layer;
		IntPair Src_Pin;
		IntPair Tar_Pin;
		string ViaName;
		// Type 3
		IntPair PatchLocate;
		I_Rect Patch;
		bool dir; // true : hor , false : ver
		wire_path(){
			path_type = UndefineValue;
			Src_Pin = make_pair(UndefineValue, UndefineValue);
			Tar_Pin = make_pair(UndefineValue, UndefineValue);
			Layer = UndefineValue;
			Patch.clear();
			PatchLocate = make_pair(UndefineValue, UndefineValue);
			ViaName.clear();
		}

		wire_path(int TYPE, int LAYER, IntPair SRC, IntPair TAR, bool DIR){
			path_type = TYPE;
			Src_Pin = make_pair(SRC.first, SRC.second);
			Tar_Pin = make_pair(TAR.first, TAR.second);
			Layer = LAYER;
			dir = DIR;
			Patch.clear();
			PatchLocate = make_pair(UndefineValue, UndefineValue);
			ViaName.clear();
		}

		void clear(){
			path_type = UndefineValue;
			Src_Pin = make_pair(UndefineValue, UndefineValue);
			Tar_Pin = make_pair(UndefineValue, UndefineValue);
			Layer = UndefineValue;
			ViaName.clear();
			Patch.clear();
			PatchLocate = make_pair(UndefineValue, UndefineValue);
		}
	};

	class IPSD_Routing_PIN{
	public:
	  IPSD_Routing_PIN(){
		  type = false;
		  pseudo = false;
		  OutSideMemory = false;
		  InsideGridNode = false;
		  index = UndefineValue;
		  target_index = UndefineValue;
		  PinShapeArea = UndefineValue;
		  IsIRoutePath = UndefineValue;
	  };

	  void clear(){
		  type = false;
		  pseudo = false;
		  OutSideMemory = false;
		  InsideGridNode = false;
		  index = UndefineValue;
		  target_index = UndefineValue;
		  PinShapeArea = UndefineValue;
		  IsIRoutePath = UndefineValue;
		  IRect_list.clear();
		  Rect_list.clear();
		  Original_PIN.clear();
		  Original_MARCO.clear();
		  LayoutNode.clear();
		  LayoutNodeOriginal.clear();
		  SynthesisSegList.clear();
		  IsolatedPinMetalSegmentLB.first.clear();
		  IsolatedPinMetalSegmentLB.second.clear();
		  IsolatedPinMetalSegmentLT.first.clear();
		  IsolatedPinMetalSegmentRB.first.clear();
		  IsolatedPinMetalSegmentRT.first.clear();
		  IsolatedPinMetalSegmentLT.second.clear();
		  IsolatedPinMetalSegmentRB.second.clear();
		  IsolatedPinMetalSegmentRT.second.clear();
	  }

	  bool type;				  //true : marco , false : pin
	  bool pseudo;				  // 1 : yes, 0 : no
	  bool OutSideMemory;
	  bool InsideGridNode;
	  int index;
	  int target_index; // for a*
	  // Pin
	  FloatPair Pin_index;
	  IntPair IPin_index;
	  string PinOrient;
	  int Layer;
	  // Shape
	  vector<I_Rect> IRect_list;
	  vector<Rect> Rect_list;
	  FloatPair Macro_Place;
	  IntPair IMacro_Place;
	  string Macro_orient;
	  // Origin Data
	  PIN Original_PIN;
	  MACRO Original_MARCO;

	  // pin x,y on maze grid
	  // ====================
	  // single pin location on steiner tree
	  int PinShapeArea;
	  int IsIRoutePath;
	  vector<RipUpNode> LayoutNode;
	  vector<RipUpNode> LayoutNodeOriginal;
	  vector<RipUpNode> LayoutNodeTRUE;
	  vector<vector<RipUpNode>> SynthesisSegList;

	  pair<vector<wire_path>, vector<wire_path>> IsolatedPinMetalSegmentLB;
	  pair<vector<wire_path>, vector<wire_path>> IsolatedPinMetalSegmentLT;
	  pair<vector<wire_path>, vector<wire_path>> IsolatedPinMetalSegmentRB;
	  pair<vector<wire_path>, vector<wire_path>> IsolatedPinMetalSegmentRT;
	};

	

	class PathAndEnc{
	public:
		vector<wire_path> path;
		vector<wire_path> m1_path;
		I_Rect Enclosure; // for hit metal
		I_Rect m1_Enclosure;
		pair<int, int> via_type;
		PathAndEnc()
		{
			path.clear();
			Enclosure.clear();
			m1_path.clear();
			m1_Enclosure.clear();
			via_type = make_pair(UndefineValue, UndefineValue);
		}
	};

	class ISPD_2PinNet{
	public:
		bool IRoute_first;
		int TargetIRouteIndex;
		int Net_id;
		IntPair TwoPinIndex;
		// A*
		I_Rect A_TargetBound;

		// add by NetConstruction
		vector<RipUpNode> SrcNodes;
		vector<RipUpNode> TarNodes;
		
		vector<RipUpNode> RipUp_path; // for maze
		vector<I_Rect> SourceBound;
		vector<I_Rect> TargetBound;

		bool subtree2subtree;
		bool subtree_is_pin;
		int SubtreePinIndex;
		// dynamic construct connection
		vector<int> PreRoutedNet; // net index
		// subtree2subtree
		vector<int> SourceSubtreeNet;

		ISPD_2PinNet(){
			TwoPinIndex = make_pair(UndefineValue, UndefineValue);
			SourceBound.clear();
			TargetBound.clear();
			RipUp_path.clear();
			PreRoutedNet.clear();
			SourceSubtreeNet.clear();
			subtree2subtree = false;
			subtree_is_pin = false;
			SubtreePinIndex = -1;
			Net_id = -1;
		}

		void clear()
		{
			TwoPinIndex = make_pair(UndefineValue, UndefineValue);
			SourceBound.clear();
			TargetBound.clear();
			RipUp_path.clear();
			PreRoutedNet.clear();
			SourceSubtreeNet.clear();
			subtree2subtree = false;
			subtree_is_pin = false;
			SubtreePinIndex = -1;
			Net_id = -1;
		}

	};

	class ISPD_Subnet{
	public:
		vector<int> PinIndexSerial;
		IntPair GuideIndex;
		int Biggest2PinNetIndex;

		ISPD_Subnet()
		{
			PinIndexSerial.clear();
			GuideIndex = make_pair(-1,-1);
			Biggest2PinNetIndex = -1;
		}
	};

	class ISPD_Routing_Net{
	public:
		bool IRoute_first;
		vector<I_Rect> GUIDE_list;
		vector<IPSD_Routing_PIN> PIN_list;
		vector<int> IRoute_PIN_list;
		Net_Name Original_Data;
		vector <wire_path> WireList;
		// for maze
		vector<IRoute *> IRoutelist;
		vector<IntPair> IRoutePair; // IRoutelist index pair
		vector<ISPD_2PinNet> TwoPinNetList;

		// merge path
		vector<vector<RipUpNode> *> Merged_path; // point to RipUp_path

		vector<ISPD_Subnet> SubnetList; // serial != net index
	};

	class Enc_Relation{
		public:
			// these two spacing mean center to center spacing
			int same_horizontal_spacing; 
			int same_vertical_spacing;
			Enc_Relation(){
				same_horizontal_spacing = UndefineValue;
				same_vertical_spacing = UndefineValue;
			}
			void clear(){
				same_horizontal_spacing = UndefineValue;
				same_vertical_spacing = UndefineValue;
			}
	};

	class EncId_HashTable{
		public:
			vector < pair <int , int> > Enc_Num; // layer dim , first: falling , second: raising
			EncId_HashTable(vector < vector <Parser::Via> > &via_type_list,int Metal_Layer_Num){
				Enc_Num.resize(Metal_Layer_Num);
				Enc_Num.at(0).first = 0;
				Enc_Num.at(0).second = via_type_list.at(0).size();
				for(int i = 1; i < via_type_list.size(); i++){
					Enc_Num.at(i).first = via_type_list.at(i - 1).size();
					Enc_Num.at(i).second = via_type_list.at(i).size();
				}
				Enc_Num.at(Metal_Layer_Num - 1).first = via_type_list.at(Metal_Layer_Num - 2).size();
			}
			EncId_HashTable(){
				;
			}
			int GetViaBitID(pair<int,int> via_type, bool up_down /* true: up , false:down*/,int cur_layer){
				if(up_down == true){ // falling
					return via_type.second;
				}
				else{
					return Enc_Num.at(cur_layer).first + via_type.second;
				}
			}
	};

	class ViaTypeBit{
		private:
			//bitset<VIA_TYPE_BIT> via_type;
			int8_t via_type;
		public:
			int getType(){
				//return (int)(via_type.to_ulong());
				return (int)via_type;
			}
			void SetType(int8_t via_type_int){
				//bitset<VIA_TYPE_BIT> new_type((unsigned long)via_type_int);
				//via_type = new_type;
				via_type = via_type_int;
			}
	};

	class Real_Coor_Table
	{
		public:
		vector< pair < vector<int> /* x coor */, vector<int> /* y coor */ > > Coor_Table;
		Real_Coor_Table() { ; }
		tuple<int, int, int> Index2Coor(int index_x, int index_y, int index_z)
		{
			if (index_z >= Coor_Table.size()){
				printf("(Real_Coor_Table) : index_z out of range (%d)\n", index_z);
				exit(1);
			}
			if (index_x >= Coor_Table.at(index_z).first.size())
			{
				printf("(Real_Coor_Table) : index_x out of range (%d)\n", index_x);
				exit(1);
			}
			if (index_y >= Coor_Table.at(index_z).second.size())
			{
				printf("(Real_Coor_Table) : index_y out of range (%d)\n", index_y);
				exit(1);
			}
			int x = Coor_Table.at(index_z).first.at(index_x);
			int y = Coor_Table.at(index_z).second.at(index_y);
			return make_tuple(x, y, index_z);
		}
		void Table_Status_Log(){
			printf("Coor_Table.size() : %d\n", Coor_Table.size());
			for (int i = 0; i < Coor_Table.size(); i++)
			{
				printf("Layer (%d) X size(%d)\n",i ,Coor_Table.at(i).first.size());
				printf("Layer (%d) Y size(%d)\n",i, Coor_Table.at(i).second.size());
			}
			int pause;
			cin >> pause;
		}
	};

	class RaisingFalling_Connection_Table{
		public:
			vector< pair < vector<int> /* raise x index */, vector<int> /* raise y index  */ > > Raising_Table;
			vector< pair < vector<int> /* fall x index  */, vector<int> /* fall y index */ > > Falling_Table;

		RaisingFalling_Connection_Table(){;}
		void SetTable(Real_Coor_Table &coor_table)
		{
			Raising_Table.resize(coor_table.Coor_Table.size());
			Falling_Table.resize(coor_table.Coor_Table.size());
			for (int i = 0; i < coor_table.Coor_Table.size(); i++)
			{
				Raising_Table.at(i).first.resize(coor_table.Coor_Table.at(i).first.size());
				Falling_Table.at(i).first.resize(coor_table.Coor_Table.at(i).first.size());
				Raising_Table.at(i).second.resize(coor_table.Coor_Table.at(i).second.size());
				Falling_Table.at(i).second.resize(coor_table.Coor_Table.at(i).second.size());
			}
		}
		tuple<int, int, int> RaisingNode_Index(int index_x, int index_y, int index_z)
		{
			int x = Raising_Table.at(index_z).first.at(index_x);
			int y = Raising_Table.at(index_z).second.at(index_y);
			return make_tuple(x, y, index_z + 1);
		}
		tuple<int, int, int> FallingNode_Index(int index_x, int index_y, int index_z)
		{
			int x = Falling_Table.at(index_z).first.at(index_x);
			int y = Falling_Table.at(index_z).second.at(index_y);
			return make_tuple(x, y, index_z - 1);
		}
	};

	class Design
	{
	  public:
		// user input
		string Lef_File, Def_File, Guide_File, Output_File;
		int Thread_num;

		int ISPD_OFFSET;
		int Metal_Layer_Num;
		int Via_Layer_Num;
		int GCell_Width, GCell_Height;
		int GCell_Original_x, GCell_Original_y;
		Rect Die_Area;
		vector<LayerInfo> Layer_list;
		vector<MACRO> Macro_list;
		vector<Component> Component_list;
		vector<Via> Via_list;
		vector<Row> Row_list;
		vector<Track> Track_list;
		vector<Net_Name> Net_Name_list;
		vector<PIN> Routing_Pin_list;
		vector<I_Rect> AllShapes;
		vector<I_Rect> AllOBSs;
		vector<int> AllShapes_NetId;
		// Data Structure For Routing
		vector<ISPD_Routing_Net> ispd_routing_net;
		vector<pair<Parser::Track, Parser::Track>> ispd_track_list;
		unordered_map<string, int> ComponentTable;
		unordered_map<string, int> RoutingPinTable;
		unordered_map<string, int> MacroTable;
		vector<unordered_map<string, int>> MacroPinTable;
		vector<tuple<int, int, int>> SortedShapesList; // area, net, pin_index

		Design()
		{
			Metal_Layer_Num = 0;
			Via_Layer_Num = 0;
			ISPD_OFFSET = 2000;
		}
		int ConponentId2MacroIndex(string component_id)
		{
			auto iter = ComponentTable.find(component_id);
			int C_id = iter->second;
			Component this_component = this->Component_list[C_id];
			string component_name = this_component.Name;
			iter = MacroTable.find(component_name);
			return iter->second;
		}

		int ConponentId2PinIndex(string component_id)
		{
			auto iter = RoutingPinTable.find(component_id);
			return iter->second;
		}

		void Design_Log()
		{
			printf("*********************************************\n");
			printf("Design info : \n");
			printf("Die Area : LB(%.2f,%.2f) RT(%.2f,%.2f)\n", Die_Area.LB.first, Die_Area.LB.second, Die_Area.RT.first, Die_Area.RT.second);
			printf("Layer Information\n");
			printf("Layer Num : %d\n", this->Layer_list.size());
			printf("Metal Layer Num : %d\n", this->Metal_Layer_Num);
			printf("Via Layer Num : %d\n", this->Via_Layer_Num);
			for (int i = 0; i < this->Layer_list.size(); i++)
			{
				if (this->Layer_list[i].Type)
				{ // Metal
					printf("	Layer%d:: Layer Name : %s\n", i, this->Layer_list[i].metalLayer.Name.c_str());
					printf("	Layer%d:: Layer Dir : %d\n", i, this->Layer_list[i].metalLayer.DIRECTION);
					printf("	Layer%d:: Layer Spacing : %.2f\n", i, this->Layer_list[i].metalLayer.SPACING);
					printf("	Layer%d:: Layer Area : %.2f\n", i, this->Layer_list[i].metalLayer.AREA);
					printf("	Layer%d:: Layer Pitch : (%.2f , %.2f)\n", i,
						   this->Layer_list[i].metalLayer.PITCH.first, this->Layer_list[i].metalLayer.PITCH.second);
					printf("	Layer%d:: Layer Width : %.2f\n", i, this->Layer_list[i].metalLayer.WIDTH);
					printf("	Layer%d:: Layer END OF LINE-Spacing: %.2f\n", i, this->Layer_list[i].metalLayer.EOL.SPACING);
					printf("	Layer%d:: Layer END OF LINE-EOL width: %.2f\n", i, this->Layer_list[i].metalLayer.EOL.ENDOFLINE);
					printf("	Layer%d:: Layer END OF LINE-Within : %.2f\n", i, this->Layer_list[i].metalLayer.EOL.WITHIN);
					printf("	Layer%d:: Layer Min Width : %.2f\n", i, this->Layer_list[i].metalLayer.MINWIDTH);
					printf("	Layer%d:: Layer PARALLELRUNLENGTH : %.2f\n", i, this->Layer_list[i].metalLayer.spacing_table.PARALLELRUNLENGTH);
					for (int j = 0; j < this->Layer_list[i].metalLayer.spacing_table.WIDTH.size(); j++)
					{
						printf("	Layer%d:: Layer PARALLELRUNLENGTH (Width,Spcing): (%.2f,%.2f)\n", i, this->Layer_list[i].metalLayer.spacing_table.WIDTH[j].first, this->Layer_list[i].metalLayer.spacing_table.WIDTH[j].second);
					}
				}
				else
				{
					printf("	Layer%d:: Layer Name : %s\n", i, this->Layer_list[i].viaLayer.Name.c_str());
					printf("	Layer%d:: Layer TYPE : %s\n", i, this->Layer_list[i].viaLayer.TYPE.c_str());
					printf("	Layer%d:: Via Cut Spacing : %.2f\n", i, this->Layer_list[i].viaLayer.CUT_SPACING);
				}
				printf("-----------------------------------------\n");

			}
			printf("Via Information\n");
			printf("Via Kind Num : %d\n", this->Via_list.size());
			for (int i = 0; i < this->Via_list.size(); i++){
				printf("	Via%d:: Via Name : %s\n", i, this->Via_list[i].Name.c_str());
				printf("	Via%d:: BotLayer Name : %s Rect LB(%.2f,%.2f) RT(%.2f,%.2f)\n", i, this->Via_list[i].BotLayerName.c_str()
					, this->Via_list[i].BotLayerRect.LB.first, this->Via_list[i].BotLayerRect.LB.second, this->Via_list[i].BotLayerRect.RT.first, this->Via_list[i].BotLayerRect.RT.second);
				printf("	Via%d:: ViaLayer Name : %s Rect LB(%.2f,%.2f) RT(%.2f,%.2f)\n", i, this->Via_list[i].ViaLayerName.c_str()
					, this->Via_list[i].ViaLayerRect.LB.first, this->Via_list[i].ViaLayerRect.LB.second, this->Via_list[i].ViaLayerRect.RT.first, this->Via_list[i].ViaLayerRect.RT.second);
				printf("	Via%d:: TopLayer Name : %s Rect LB(%.2f,%.2f) RT(%.2f,%.2f)\n", i, this->Via_list[i].TopLayerName.c_str()
					, this->Via_list[i].TopLayerRect.LB.first, this->Via_list[i].TopLayerRect.LB.second, this->Via_list[i].TopLayerRect.RT.first, this->Via_list[i].TopLayerRect.RT.second);
				printf("-----------------------------------------\n");
			}
			printf("Macro Information\n");
			printf("Macro Num : %d\n", this->Macro_list.size());
			for (int i = 0; i < this->Macro_list.size(); i++){
				printf("	Macro%d:: Macro Name : %s\n", i, this->Macro_list[i].Name.c_str());
				printf("	Macro%d:: Macro Class : %s\n", i, this->Macro_list[i].CLASS.c_str());
				printf("	Macro%d:: Macro SIZE %.2f BY %.2f\n", i, this->Macro_list[i].SIZE_BY.first, this->Macro_list[i].SIZE_BY.second);
				printf("	Macro%d:: Macro ORIGIN (%.2f, %.2f)\n", i, this->Macro_list[i].ORIGIN.first, this->Macro_list[i].ORIGIN.second);
				for (int j = 0; j < this->Macro_list[i].Pin_list.size(); j++){
					printf("		Macro%d::PIN%d:: Pin Name : %s\n", i, j, this->Macro_list[i].Pin_list[j].Name.c_str());
					printf("		Macro%d::PIN%d:: Pin DIRECTION : %s\n", i, j, this->Macro_list[i].Pin_list[j].DIRECTION.c_str());
					printf("		Macro%d::PIN%d:: Pin USE : %s\n", i, j, this->Macro_list[i].Pin_list[j].USE.c_str());
					printf("		Macro%d::PIN%d:: Pin Shape : %s\n", i, j, this->Macro_list[i].Pin_list[j].Shape.c_str());
					printf("		Macro%d::PIN%d:: Port LAYER : %s\n", i, j, this->Macro_list[i].Pin_list[j].port.LAYER.c_str());
					for (int k = 0; k < this->Macro_list[i].Pin_list[j].port.rect_list.size(); k++)
						printf("			Macro%d::PIN%d:: Rect%d LB(%.3f, %.3f) RT(%.3f, %.3f)\n", i, j, k,
						this->Macro_list[i].Pin_list[j].port.rect_list[k].LB.first, this->Macro_list[i].Pin_list[j].port.rect_list[k].LB.second,
						this->Macro_list[i].Pin_list[j].port.rect_list[k].RT.first, this->Macro_list[i].Pin_list[j].port.rect_list[k].RT.second);

					if (this->Macro_list[i].Pin_list[j].Name == "OBS" )
					{
						int pause;
						cin >> pause;
					}
				}
				printf("-----------------------------------------\n");
			}
			printf("Row Information\n");
			printf("Row Num : %d\n", this->Row_list.size());
			for (int i = 0; i < this->Row_list.size(); i++){
				printf("	Row%d:: Row Name : %s\n", i, this->Row_list[i].Name.c_str());
				printf("	Row%d:: Row Macro : %s\n", i, this->Row_list[i].Macro.c_str());
				printf("	Row%d:: Row Start Index : (%d,%d)\n", i, this->Row_list[i].start_index.first, this->Row_list[i].start_index.second);
				printf("	Row%d:: Row Dir : %s\n", i, this->Row_list[i].Dir.c_str());
				printf("	Row%d:: Row DO %d By %d\n", i, this->Row_list[i].Do_By.first, this->Row_list[i].Do_By.second);
				printf("	Row%d:: Row Step %d %d\n", i, this->Row_list[i].Step.first, this->Row_list[i].Step.second);
				printf("-----------------------------------------\n");
			}
			printf("Track Information\n");
			printf("Track Num : %d\n", this->Track_list.size());
			for (int i = 0; i < this->Track_list.size(); i++){
				printf("	Track%d:: Track Layer : %s\n", i, this->Track_list[i].Layer.c_str());
				printf("	Track%d:: Track Dir : %s\n", i, this->Track_list[i].Dir.c_str());
				printf("	Track%d:: Track Start Index : %d\n", i, this->Track_list[i].start_index);
				printf("	Track%d:: Track DO %d\n", i, this->Track_list[i].Do);
				printf("	Track%d:: Track Step %d\n", i, this->Track_list[i].Step);
				printf("-----------------------------------------\n");
			}
			printf("Component Information\n");
			printf("Component Num : %d\n", this->Component_list.size());
			for (int i = 0; i < this->Component_list.size(); i++){
				printf("	Component%d:: Component Name : %s\n", i, this->Component_list[i].Name.c_str());
				printf("	Component%d:: Component ID : %s\n", i, this->Component_list[i].ID.c_str());
				printf("	Component%d:: Component Index : (%d,%d)\n", i, this->Component_list[i].index.first, this->Component_list[i].index.second);
				printf("	Component%d:: Component Dir : %s\n", i, this->Component_list[i].Dir.c_str());
				printf("-----------------------------------------\n");
			}
			printf("PIN Information\n");
			printf("PIN Num : %d\n", this->Routing_Pin_list.size());
			for (int i = 0; i < this->Routing_Pin_list.size(); i++){
				printf("	Routing_Pin%d:: Routing_Pin : %s\n", i, this->Routing_Pin_list[i].Name.c_str());
				printf("	Routing_Pin%d:: DIRECTION : %s\n", i, this->Routing_Pin_list[i].DIRECTION.c_str());
				printf("	Routing_Pin%d:: Net_belong : %s\n", i, this->Routing_Pin_list[i].Net_belong.c_str());
				printf("	Routing_Pin%d:: Layer : %s\n", i, this->Routing_Pin_list[i].Layer.c_str());
				printf("	Routing_Pin%d:: Layer Bound LB(%.3f, %.3f) RT(%.3f, %.3f)\n", i, this->Routing_Pin_list[i].LayerBound.LB.first, this->Routing_Pin_list[i].LayerBound.LB.second,
					this->Routing_Pin_list[i].LayerBound.RT.first, this->Routing_Pin_list[i].LayerBound.RT.second);
				printf("	Routing_Pin%d:: USE : %s\n", i, this->Routing_Pin_list[i].USE.c_str());
				printf("	Routing_Pin%d:: Index : (%d,%d) %s\n", i, this->Routing_Pin_list[i].Place_Index.first, this->Routing_Pin_list[i].Place_Index.second, this->Routing_Pin_list[i].Orient.c_str());
				printf("-----------------------------------------\n");
			}
			printf("Net Information\n");
			printf("Net Num : %d\n", this->Net_Name_list.size());
			for (int i = 0; i < this->Net_Name_list.size(); i++){
				printf("	Net%d:: Net Net_name : %s\n", i, this->Net_Name_list[i].Net_name.c_str());
				for (int j = 0; j < Net_Name_list[i].CPN_list.size(); j++){
					printf("		Net%d:: Component %d ID : %s\n", i, j, this->Net_Name_list[i].CPN_list[j].Component_Name.c_str());
					printf("		Net%d:: Component %d Pin : %s\n", i, j, this->Net_Name_list[i].CPN_list[j].Pin_Name.c_str());
				}
				printf("-----------------------------------------\n");
			}


			printf("*********************************************\n");

		}
		void DebugPrint(char *str)
		{
			cout << str << endl;
		}
		void DebugPrint(int &INTEGER)
		{
			cout << INTEGER << endl;
		}
	};
}



//extern Design This_Design;

#endif