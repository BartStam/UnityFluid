using Unity.Mathematics;
using UnityEngine;

public class Particle {
	private GameObject gameObject;
	public float3 proposedPosition;
	public float3 velocity;
	public float3 Position {
		get { return gameObject.transform.position; }
		set { gameObject.transform.position = value; }
	}

	// Particle state
	public float density;
	public float pressure;

	// Forces
	public float3 externalForce;
	public float3 pressureForce;
	public float3 viscosityForce;
	public float3 CombinedForce {
		get {
			return externalForce + pressureForce + viscosityForce;
		}
	}

	public Particle(GameObject gameObject) {
		this.gameObject = gameObject;
	}
}
