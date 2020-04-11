using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Particle {
	private GameObject gameObject;
	public Vector3 proposedPosition;
	public Vector3 velocity;
	public Vector3 Position {
		get { return gameObject.transform.position; }
		set { gameObject.transform.position = value; }
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
		this.gameObject = gameObject;
	}
}
