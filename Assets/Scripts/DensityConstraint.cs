//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;

//public class DensityConstraint {
//	private Particle particle;

//	public DensityConstraint(Particle particle) {
//		this.particle = particle;
//	}

//	public void Step() {
//		particle.density = 0.0f;
//		particle.pressureForce = Vector3.zero;
//		particle.viscosityForce = Vector3.zero;

//		foreach (Particle p in Controller.particles) {
//			particle.density += Kernels.Poly6(particle.Position - p.Position, Controller.smoothingRadius);
//		}

//		particle.pressure = Controller.gasConstant * (particle.density - Controller.restDensity);
//		particle.gravityForce = Controller.gravity * particle.density;

//		foreach (Particle p in Controller.particles) {
//			if (particle.Equals(p)) { continue; }

//			particle.pressureForce -= ((particle.pressure + p.pressure) / (2.0f * p.density)) * Kernels.SpikyGradient(particle.Position - p.Position, Controller.smoothingRadius);
//			particle.viscosityForce += Controller.viscosity * ((p.velocity - particle.velocity) / p.density) * Kernels.ViscosityLaplacian(particle.Position - p.Position, Controller.smoothingRadius);
//		}
//	}
//}
