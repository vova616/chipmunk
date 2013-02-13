package chipmunk

import (
	"github.com/vova616/chipmunk/vect"
	"time"
)

type SpatialIndexQueryFunc func(a, b Indexable)
type ReindexShapesFunc func(a, b *Shape, space *Space)
type HashSetIterator func(node *Node)

type TreeType byte

type SpatialIndex struct {
	SpatialIndexClass

	staticIndex, dynamicIndex SpatialIndexClass
}

func SpatialIndexCollideStatic(dynamicIndex, staticIndex *SpatialIndex, fnc SpatialIndexQueryFunc) {
	if staticIndex.Count() > 0 {
		dynamicIndex.Each(func(node *Node) {
			staticIndex.Query(node.obj, node.obj.AABB(), fnc)
		})
	}
}

func NewSpartialIndex(class SpatialIndexClass, staticIndex *SpatialIndex) (index *SpatialIndex) {

	if staticIndex != nil {
		index = &SpatialIndex{class, staticIndex.SpatialIndexClass, nil}
	} else {
		index = &SpatialIndex{class, nil, nil}
	}

	if staticIndex != nil {
		if staticIndex.dynamicIndex != nil {
			panic("This static index is already associated with a dynamic index.")
		}
		staticIndex.dynamicIndex = index
	}

	return
}

type Indexable interface {
	Hashable
	AABB() AABB
	Shape() *Shape
	Velocity() (vect.Vect, bool)
}

type Data interface{}

type Hashable interface {
	Hash() HashValue
}

type SpatialIndexClass interface {
	Destroy()

	Count() int
	Each(fnc HashSetIterator)

	Contains(obj Indexable) bool
	Insert(obj Indexable)
	Remove(obj Indexable)

	Reindex()
	ReindexObject(obj Indexable)
	ReindexQuery(fnc SpatialIndexQueryFunc)

	Stamp() time.Duration

	Query(obj Indexable, aabb AABB, fnc SpatialIndexQueryFunc)
	SegmentQuery(obj Indexable, a, b vect.Vect, t_exit vect.Float, fnc func())
}
