package chipmunk

import (
	"reflect"
	"sync/atomic"
)

var hashCounter = uint64(0)

type HashValue uint32

const hashCoef = HashValue(3344921057)

func hashPair(a, b HashValue) HashValue {
	return (a * hashCoef) ^ (b * hashCoef)
}

type HashPair struct {
	A *Shape
	B *Shape
}

func newPair(A *Shape, B *Shape) HashPair {
	if A.Hash() > B.Hash() {
		return HashPair{A, B}
	}
	return HashPair{B, A}

}

type DefaultHash struct {
	hash HashValue
}

func ToHash(ptr interface{}) HashValue {
	return HashValue(reflect.ValueOf(ptr).Pointer())
}

func (h *DefaultHash) Hash() HashValue {
	if h.hash == 0 {
		h.hash = HashValue(atomic.AddUint64(&hashCounter, 1))
		if h.hash == 0 {
			panic("Hash overflowed")
		}
	}
	return h.hash
}

func (h *DefaultHash) Reset() {
	h.hash = 0
}
