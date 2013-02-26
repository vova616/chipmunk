package chipmunk

import (
	//"container/list"
	"github.com/vova616/chipmunk/vect"
	//"log"
	"time"
)

var VoidQueryFunc = func(a, b Indexable) {}

type HashSet map[HashValue]*Node

func (set HashSet) Each(fnc HashSetIterator) {
	for _, node := range set {
		fnc(node)
	}
}

type BBTree struct {
	SpatialIndexClass
	SpatialIndex *SpatialIndex

	leaves HashSet
	root   *Node

	pairBuffer []*Pair
	nodeBuffer []*Node

	stamp time.Duration
}

type Children struct {
	A, B *Node
}

type Leaf struct {
	stamp time.Duration
	pairs *Pair
}

type node struct {
	Children
	Leaf
}

type Node struct {
	obj    Indexable
	bb     AABB
	parent *Node

	node
}

func (node *Node) IsLeaf() bool {
	return node.obj != nil
}

func (node *Node) NodeSetA(value *Node) {
	node.A = value
	value.parent = node
}

func (node *Node) NodeSetB(value *Node) {
	node.B = value
	value.parent = node
}

func (node *Node) NodeOther(child *Node) *Node {
	if node.A == child {
		return node.B
	}
	return node.A
}

type Thread struct {
	prev *Pair
	leaf *Node
	next *Pair
}

type Pair struct {
	a, b Thread
}

func NewBBTree(staticIndex *SpatialIndex) *SpatialIndex {
	tree := &BBTree{}
	tree.leaves = make(map[HashValue]*Node)

	tree.SpatialIndex = NewSpartialIndex(tree, staticIndex)
	tree.pairBuffer = make([]*Pair, 0)
	tree.nodeBuffer = make([]*Node, 0)
	return tree.SpatialIndex
}

func (tree *BBTree) Count() int {
	return len(tree.leaves)
}

func (tree *BBTree) NewLeaf(obj Indexable) *Node {
	node := tree.NodeFromPool()
	node.obj = obj
	node.bb = tree.GetBB(obj)

	return node
}

func (tree *BBTree) NodeNew(a, b *Node) *Node {
	node := tree.NodeFromPool()
	node.bb = CombinePtr(&a.bb, &b.bb)

	node.NodeSetA(a)
	node.NodeSetB(b)

	return node
}

func (tree *BBTree) GetBB(obj Indexable) AABB {
	v, ok := obj.Velocity()
	if ok {
		bb := obj.AABB()
		coef := vect.Float(0.1)

		l := bb.Lower.X
		b := bb.Lower.Y
		r := bb.Upper.X
		t := bb.Upper.Y

		x := (r - l) * coef
		y := (t - b) * coef

		v = vect.Mult(v, 0.1)

		return NewAABB(l+vect.FMin(-x, v.X), b+vect.FMin(-y, v.Y), r+vect.FMax(x, v.X), t+vect.FMax(y, v.Y))
	}

	return obj.AABB()
}

func (tree *BBTree) SubtreeInsert(subtree, leaf *Node) *Node {
	if subtree == nil {
		return leaf
	} else if subtree.IsLeaf() {
		return tree.NodeNew(leaf, subtree)
	}

	cost_a := subtree.B.bb.Area() + MergedAreaPtr(&subtree.A.bb, &leaf.bb)
	cost_b := subtree.A.bb.Area() + MergedAreaPtr(&subtree.B.bb, &leaf.bb)

	if cost_a == cost_b {
		cost_a = ProximityPtr(&subtree.A.bb, &leaf.bb)
		cost_b = ProximityPtr(&subtree.B.bb, &leaf.bb)
	}

	if cost_b < cost_a {
		subtree.NodeSetB(tree.SubtreeInsert(subtree.B, leaf))
	} else {
		subtree.NodeSetA(tree.SubtreeInsert(subtree.A, leaf))
	}

	subtree.bb = CombinePtr(&subtree.bb, &leaf.bb)

	return subtree
}

func GetTree(index SpatialIndexClass) *BBTree {
	if index != nil {
		tree, _ := index.(*BBTree)
		return tree
	}
	return nil
}

func (tree *BBTree) GetMasterTree() SpatialIndexClass {
	dynamicTree := tree.SpatialIndex.dynamicIndex
	if dynamicTree != nil {
		return dynamicTree
	}
	return tree
}

func (tree *BBTree) GetMasterTreeStamp() time.Duration {
	dynamicTree := tree.SpatialIndex.dynamicIndex
	if dynamicTree != nil {
		return dynamicTree.Stamp()
	}
	return tree.stamp
}

func (tree *BBTree) Stamp() time.Duration {
	return tree.stamp
}

func GetRootIfTree(index SpatialIndexClass) *Node {
	if index != nil {
		tree, ok := index.(*BBTree)
		if ok {
			return tree.root
		}
	}
	return nil
}

func (tree *BBTree) LeafAddPairs(leaf *Node) {
	dynamicIndex := tree.SpatialIndex.dynamicIndex
	if dynamicIndex != nil {
		dynamicRoot := GetRootIfTree(dynamicIndex)
		if dynamicRoot != nil {
			dynamicTree := GetTree(dynamicIndex)
			context := MarkContext{dynamicTree, nil, nil}
			context.MarkLeafQuery(dynamicRoot, leaf, true)
		}
	} else {
		staticRoot := GetRootIfTree(tree.SpatialIndex.staticIndex)
		context := MarkContext{tree, staticRoot, VoidQueryFunc}
		context.MarkLeaf(leaf)
	}
}

func (tree *BBTree) IncrementStamp() {
	dynamicTree := GetTree(tree.SpatialIndex.dynamicIndex)
	if dynamicTree != nil {
		dynamicTree.stamp++
	} else {
		tree.stamp++
	}
}

func (tree *BBTree) Insert(obj Indexable) {

	leaf := tree.NewLeaf(obj)

	tree.leaves[obj.Hash()] = leaf

	root := tree.root
	tree.root = tree.SubtreeInsert(root, leaf)

	leaf.stamp = tree.GetMasterTree().Stamp()

	tree.LeafAddPairs(leaf)
	tree.IncrementStamp()
}

func (tree *BBTree) PairInsert(a, b *Node) {
	nextA := a.pairs
	nextB := b.pairs
	pair := tree.PairFromPool()
	b.pairs = pair
	a.pairs = pair

	*pair = Pair{Thread{nil, a, nextA}, Thread{nil, b, nextB}}

	if nextA != nil {
		if nextA.a.leaf == a {
			nextA.a.prev = pair
		} else {
			nextA.b.prev = pair
		}
	}

	if nextB != nil {
		if nextB.a.leaf == b {
			nextB.a.prev = pair
		} else {
			nextB.b.prev = pair
		}
	}
}

func (tree *BBTree) NodeRecycle(node *Node) {
	*node = Node{}
	tree.nodeBuffer = append(tree.nodeBuffer, node)
}

func (tree *BBTree) NodeFromPool() *Node {
	var node *Node
	if len(tree.nodeBuffer) > 0 {
		node, tree.nodeBuffer = tree.nodeBuffer[len(tree.nodeBuffer)-1], tree.nodeBuffer[:len(tree.nodeBuffer)-1]
	} else {
		buff := make([]*Node, 100)
		//log.Println("New node buff")
		for i, _ := range buff {
			buff[i] = &Node{}
		}
		tree.nodeBuffer = append(tree.nodeBuffer, buff...)
		node, tree.nodeBuffer = tree.nodeBuffer[len(tree.nodeBuffer)-1], tree.nodeBuffer[:len(tree.nodeBuffer)-1]
	}
	return node
}

func (tree *BBTree) PairRecycle(pair *Pair) {
	*pair = Pair{}
	tree.pairBuffer = append(tree.pairBuffer, pair)
}

func (tree *BBTree) PairFromPool() *Pair {
	var pair *Pair
	if len(tree.pairBuffer) > 0 {
		pair, tree.pairBuffer = tree.pairBuffer[len(tree.pairBuffer)-1], tree.pairBuffer[:len(tree.pairBuffer)-1]
	} else {
		buff := make([]*Pair, 100)
		//log.Println("New pair buff")
		for i, _ := range buff {
			buff[i] = &Pair{}
		}
		tree.pairBuffer = append(tree.pairBuffer, buff...)
		pair, tree.pairBuffer = tree.pairBuffer[len(tree.pairBuffer)-1], tree.pairBuffer[:len(tree.pairBuffer)-1]
	}
	return pair
}

func (tree *BBTree) NodeReplaceChild(parent, child, value *Node) {
	if parent.IsLeaf() {
		panic("Internal Error: Cannot replace child of a leaf.")
	}

	if (child == parent.A || child == parent.B) == false {
		panic("Internal Error: Node is not a child of parent.")
	}

	if parent.A == child {
		tree.NodeRecycle(parent.A)
		parent.NodeSetA(value)
	} else {
		tree.NodeRecycle(parent.B)
		parent.NodeSetB(value)
	}

	for node := parent; node != nil; node = node.parent {
		node.bb = CombinePtr(&node.A.bb, &node.B.bb)
	}
}

func (tree *BBTree) Remove(obj Indexable) {
	leaf := tree.leaves[obj.Hash()]
	delete(tree.leaves, obj.Hash())

	if leaf == nil {
		return
	}

	tree.root = tree.SubtreeRemove(tree.root, leaf)
	tree.PairsClear(leaf)
	tree.NodeRecycle(leaf)
}

func (tree *BBTree) SubtreeRemove(subtree, leaf *Node) *Node {
	if leaf == subtree {
		return nil
	}

	parent := leaf.parent
	if parent == subtree {
		other := subtree.NodeOther(leaf)
		other.parent = subtree.parent
		tree.NodeRecycle(subtree)
		return other
	}

	tree.NodeReplaceChild(parent.parent, parent, parent.NodeOther(leaf))
	return subtree
}

func ThreadUnlink(thread Thread) {
	next := thread.next
	prev := thread.prev

	if next != nil {
		if next.a.leaf == thread.leaf {
			next.a.prev = prev
		} else {
			next.b.prev = prev
		}
	}

	if prev != nil {
		if prev.a.leaf == thread.leaf {
			prev.a.next = next
		} else {
			prev.b.next = next
		}
	} else {
		thread.leaf.pairs = next
	}
}

func (tree *BBTree) PairsClear(leaf *Node) {
	pair := leaf.pairs
	leaf.pairs = nil

	for pair != nil {
		if pair.a.leaf == leaf {
			next := pair.a.next
			ThreadUnlink(pair.b)
			tree.PairRecycle(pair)
			pair = next
		} else {
			next := pair.b.next
			ThreadUnlink(pair.a)
			tree.PairRecycle(pair)
			pair = next
		}
	}
}

func LeafUpdate(leaf *Node, tree *BBTree) bool {
	root := tree.root

	if !leaf.bb.Contains(leaf.obj.AABB()) {
		leaf.bb = tree.GetBB(leaf.obj)

		root = tree.SubtreeRemove(root, leaf)
		tree.root = tree.SubtreeInsert(root, leaf)

		tree.PairsClear(leaf)
		leaf.stamp = tree.GetMasterTree().Stamp()

		return true
	}

	return false
}

func (tree *BBTree) Each(fnc HashSetIterator) {
	tree.leaves.Each(fnc)
}

func (tree *BBTree) ReindexQuery(fnc SpatialIndexQueryFunc) {
	if tree.root == nil {
		return
	}

	// LeafUpdate() may modify tree->root. Don't cache it.
	for _, node := range tree.leaves {
		LeafUpdate(node, tree)
	}

	staticIndex := GetTree(tree.SpatialIndex.staticIndex)
	var staticRoot *Node = nil
	if staticIndex != nil {
		staticRoot = staticIndex.root
	}

	context := MarkContext{tree, staticRoot, fnc}
	context.MarkSubtree(tree.root)
	if staticIndex != nil && staticRoot == nil {
		SpatialIndexCollideStatic(tree.SpatialIndex, staticIndex.SpatialIndex, fnc)
	}

	tree.IncrementStamp()
}

func (tree *BBTree) Query(obj Indexable, aabb AABB, fnc SpatialIndexQueryFunc) {
	if tree.root != nil {
		SubtreeQuery(tree.root, obj, aabb, fnc)
	}
}

func SubtreeQuery(subtree *Node, obj Indexable, bb AABB, fnc SpatialIndexQueryFunc) {
	if TestOverlap(subtree.bb, bb) {
		if subtree.IsLeaf() {
			fnc(obj, subtree.obj)
		} else {
			SubtreeQuery(subtree.A, obj, bb, fnc)
			SubtreeQuery(subtree.B, obj, bb, fnc)
		}
	}
}

type MarkContext struct {
	tree       *BBTree
	staticRoot *Node
	fnc        SpatialIndexQueryFunc
}

func (context *MarkContext) MarkSubtree(subtree *Node) {
	if subtree.IsLeaf() {
		context.MarkLeaf(subtree)
	} else {
		context.MarkSubtree(subtree.A)
		context.MarkSubtree(subtree.B)
	}
}

func (context *MarkContext) MarkLeafQuery(subtree, leaf *Node, left bool) {

	if TestOverlapPtr(&leaf.bb, &subtree.bb) {
		if subtree.IsLeaf() {
			if left {
				context.tree.PairInsert(leaf, subtree)
			} else {
				if subtree.stamp < leaf.stamp {
					context.tree.PairInsert(subtree, leaf)
				}
				context.fnc(leaf.obj, subtree.obj)
			}
		} else {
			context.MarkLeafQuery(subtree.A, leaf, left)
			context.MarkLeafQuery(subtree.B, leaf, left)
		}
	}
}

func (context *MarkContext) MarkLeaf(leaf *Node) {
	tree := context.tree
	if leaf.stamp == tree.GetMasterTreeStamp() {
		staticRoot := context.staticRoot
		if staticRoot != nil {
			context.MarkLeafQuery(staticRoot, leaf, false)
		}

		for node := leaf; node.parent != nil; node = node.parent {
			if node == node.parent.A {
				context.MarkLeafQuery(node.parent.B, leaf, true)
			} else {
				context.MarkLeafQuery(node.parent.A, leaf, false)
			}
		}
	} else {
		pair := leaf.pairs
		i := 0
		for pair != nil {
			if leaf == pair.b.leaf {
				context.fnc(pair.a.leaf.obj, leaf.obj)
				pair = pair.b.next
			} else {
				pair = pair.a.next
			}
			i++
		}
		//fmt.Println(i)
	}
}
