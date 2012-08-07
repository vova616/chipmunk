package chipmunk

import (
	. "chipmunk/vect"
)	

type SpatialIndexQueryFunc func(a,b interface{}, data interface{})
type HashSetIterator func(node *Node,  data interface{})

	
type SpatialIndex struct {
	SpatialIndexClass
	
	staticIndex, dynamicIndex SpatialIndexClass
}

func SpatialIndexCollideStatic(dynamicIndex, staticIndex *SpatialIndex, fnc SpatialIndexQueryFunc, data interface{}) {
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
	data interface{}
} 

func dynamicToStaticIter(node *Node,  data interface{}) {
	context := data.(dynamicToStaticContext)
	context.staticIndex.Query(node, node.bb, context.queryFunc, context.data)
}



type Indexable interface {
	Hashable
	AABB() AABB
	Velocity() (Vect,bool)
}

type Hashable interface {
	Hash() HashValue
}


type SpatialIndexClass interface {
	Destroy()

	Count() int
	Each(fnc HashSetIterator, data interface{})

	Contains(obj Indexable) bool
	Insert(obj Indexable) 
	Remove(obj Indexable)  

	Reindex()
	ReindexObject(obj Indexable)
	ReindexQuery(fnc SpatialIndexQueryFunc, data interface{})

	Query(obj interface{}, aabb AABB, fnc SpatialIndexQueryFunc, data interface{})
	SegmentQuery(obj interface{}, a,b Vect, t_exit Float, fnc func(), data interface{})
}
