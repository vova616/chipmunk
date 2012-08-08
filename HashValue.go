package chipmunk

import (
	"sync/atomic"
	"unsafe"
	
)	

var hashCounter = uint32(0)

const hashCoef = HashValue(3344921057)

func hashPair(a, b HashValue) HashValue {
	return (a * hashCoef) ^ (b * hashCoef)
}


type DefaultHash struct {
	hash HashValue
}

func (h *DefaultHash) Hash() HashValue {
	if h.hash == 0 {
		h.hash = HashValue(atomic.AddUint32(&hashCounter, 1))
		h.hash = HashValue(uintptr(unsafe.Pointer(h)))
		if h.hash == 0 {
			panic("Hash overflowed")
		} 
		return h.hash
	}
	return h.hash
}