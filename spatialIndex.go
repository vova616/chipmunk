package chipmunk

import (
	. "chipmunk/vect"
	"time"
)	

type SpatialIndexQueryFunc func(a,b Indexable, data Data)
type ReindexShapesFunc func(a,b *Shape, space *Space)
type HashSetIterator func(node *Node, data Data)

	
type SpatialIndex struct {
	SpatialIndexClass
	
	staticIndex, dynamicIndex SpatialIndexClass
}

func SpatialIndexCollideStatic(dynamicIndex, staticIndex *SpatialIndex, fnc SpatialIndexQueryFunc, data Data) {
	if staticIndex.Count() > 0 {
		context := dynamicToStaticContext{staticIndex, fnc, data};
		dynamicIndex.Each(dynamicToStaticIter, context)
	}
}


func NewSpartialIndex(class SpatialIndexClass, staticIndex *SpatialIndex) (index *SpatialIndex) {
	
	index = &SpatialIndex{class,staticIndex,nil}
	
	if (staticIndex != nil) {
		if (staticIndex.dynamicIndex != nil) {
			panic("This static index is already associated with a dynamic index.")
		}
		staticIndex.dynamicIndex = index
	} 
	
	return
}

type dynamicToStaticContext struct {
	staticIndex *SpatialIndex 
	queryFunc SpatialIndexQueryFunc
	data Data
} 

func (d dynamicToStaticContext) Space() *Space {
	return nil
}

func dynamicToStaticIter(node *Node,  data Data) {
	context := data.(dynamicToStaticContext)
	context.staticIndex.Query(node.obj, node.obj.AABB(), context.queryFunc, context.data)
}



type Indexable interface {
	Hashable
	AABB() AABB
	Shape() *Shape
	Velocity() (Vect,bool)
}

type Data interface {}


type Hashable interface {
	Hash() HashValue
}


type SpatialIndexClass interface {
	Destroy()

	Count() int
	Each(fnc HashSetIterator, data Data)

	Contains(obj Indexable) bool
	Insert(obj Indexable) 
	Remove(obj Indexable)  

	Reindex()
	ReindexObject(obj Indexable)
	ReindexQuery(fnc SpatialIndexQueryFunc, data Data)
	
	Stamp() time.Duration
	
	Query(obj Indexable, aabb AABB, fnc SpatialIndexQueryFunc, data Data)
	SegmentQuery(obj Indexable, a,b Vect, t_exit Float, fnc func(), data Data)
}
