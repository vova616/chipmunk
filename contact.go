package chipmunk

import (
	"github.com/vova616/chipmunk/vect"
)

type Contact struct {
	p, n vect.Vect
	dist vect.Float

	r1, r2               vect.Vect
	nMass, tMass, bounce vect.Float

	jnAcc, jtAcc, jBias vect.Float
	bias                vect.Float

	hash HashValue
}

func (con *Contact) reset(pos, norm vect.Vect, dist vect.Float, hash HashValue) {
	con.p = pos
	con.n = norm
	con.dist = dist
	con.hash = hash

	con.jnAcc = 0.0
	con.jtAcc = 0.0
	con.jBias = 0.0
}

func (con *Contact) Normal() vect.Vect {
	return con.n
}

func (con *Contact) Position() vect.Vect {
	return con.p
}
