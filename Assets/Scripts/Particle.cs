using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Particle {
	private Rigidbody rigidbody;
	public Vector3 proposedPosition;
	public Vector3 velocity;
	public Vector3 Position {
		get { return rigidbody.position; }
		set { rigidbody.position = value; }
	}

	// Particle state
	public float density;
	public float pressure;

	// Forces
	public Vector3 externalForce;
	public Vector3 pressureForce;
	public Vector3 viscosityForce;
	public Vector3 CombinedForce {
		get {
			return externalForce + pressureForce + viscosityForce;
		}
	}

	public Particle(GameObject gameObject) {
		rigidbody = gameObject.GetComponent<Rigidbody>();
	}
}
