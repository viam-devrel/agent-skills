package main

import (
	"fmt"
	"os"
	"strconv"
	"strings"

	"go.viam.com/rdk/referenceframe"
)

func main() {
	var atFloats []float64
	haveAt := false
	var files []string

	for i := 1; i < len(os.Args); i++ {
		arg := os.Args[i]
		if arg == "--at" {
			i++
			if i >= len(os.Args) {
				fmt.Println("REJECT  --at requires a comma-separated list of floats")
				os.Exit(1)
			}
			haveAt = true
			for _, tok := range strings.Split(os.Args[i], ",") {
				v, err := strconv.ParseFloat(strings.TrimSpace(tok), 64)
				if err != nil {
					fmt.Printf("REJECT  --at value %q is not a float: %v\n", tok, err)
					os.Exit(1)
				}
				atFloats = append(atFloats, v)
			}
			continue
		}
		files = append(files, arg)
	}

	for _, p := range files {
		m, err := referenceframe.KinematicModelFromFile(p, "probe")
		if err != nil {
			fmt.Printf("REJECT  %-50s  %v\n", shorten(p), err)
			continue
		}
		if !haveAt {
			fmt.Printf("ACCEPT  %-50s  DoF=%d\n", shorten(p), len(m.DoF()))
			continue
		}

		if len(atFloats) != len(m.DoF()) {
			fmt.Printf("REJECT  %-50s  --at has %d value(s), model DoF=%d\n", shorten(p), len(atFloats), len(m.DoF()))
			continue
		}
		inputs := make([]referenceframe.Input, len(atFloats))
		copy(inputs, atFloats)

		pose, err := m.Transform(inputs)
		if err != nil {
			fmt.Printf("REJECT  %-50s  Transform error: %v\n", shorten(p), err)
			continue
		}
		pt := pose.Point()
		q := pose.Orientation().Quaternion()
		fmt.Printf(
			"POSE    %-50s  point_mm=[%.9f %.9f %.9f]  quat=[%.9f %.9f %.9f %.9f]\n",
			shorten(p), pt.X, pt.Y, pt.Z, q.Real, q.Imag, q.Jmag, q.Kmag,
		)
	}
}

func shorten(p string) string {
	if len(p) > 48 {
		return "..." + p[len(p)-45:]
	}
	return p
}
