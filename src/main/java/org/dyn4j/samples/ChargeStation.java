/*
 * Copyright (c) 2010-2022 William Bittle  http://www.dyn4j.org/
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted 
 * provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice, this list of conditions 
 *     and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
 *     and the following disclaimer in the documentation and/or other materials provided with the 
 *     distribution.
 *   * Neither the name of dyn4j nor the names of its contributors may be used to endorse or 
 *     promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.dyn4j.samples;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Circle;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.Camera;
import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.samples.framework.SimulationFrame;

/**
 * Class used to show a simple example of using the dyn4j project using
 * Java2D for rendering.
 * <p>
 * This class can be used as a starting point for projects.
 * 
 * @author William Bittle
 * @version 5.0.0
 * @since 3.0.0
 */
public class ChargeStation extends SimulationFrame {
	/** The serial version id */
	private static final long serialVersionUID = 5663760293144882635L;

	/**
	 * Default constructor for the window
	 */
	public ChargeStation() {
		super("Graphics2D Example");

	}

	/**
	 * Creates game objects and adds them to the world.
	 * <p>
	 * Basically the same shapes from the Shapes test in
	 * the TestBed.
	 */
	protected void initializeWorld() {
		// create all your bodies/joints

		// units are inches, as specified in
		// https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/TeamVersions/Drawings/TE-23301-HalfChargeStation-Dwg.pdf

		// create the floor
		Rectangle floorRect = new Rectangle(100.0, 1.0);
		SimulationBody floor = new SimulationBody();
		floor.addFixture(new BodyFixture(floorRect));
		floor.setMass(MassType.INFINITE);
		// move the floor down a bit
		// floor.translate(0.0, -4.0);
		floor.translate(0.0, -0.5); // so floor surface is at zero
		this.world.addBody(floor);

		// TE-23305 (hinge base)
		Rectangle te23305_shape = new Rectangle(18, 0.47);
		SimulationBody te23305 = new SimulationBody();
		te23305.addFixture(te23305_shape);
		te23305.setMass(MassType.INFINITE);
		te23305.translate(0.0, 0.47 / 2);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23305);

		// TE-23317 (hinge stand)
		Rectangle te23317_shape = new Rectangle(1.5, 3.94);
		SimulationBody te23317 = new SimulationBody();
		te23317.addFixture(te23317_shape);
		te23317.setMass(MassType.INFINITE);
		te23317.translate(-1.75, 3.94 / 2 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23317);

		// TE-23317 (hinge stand)
		Rectangle te23317_2_shape = new Rectangle(1.5, 3.94);
		SimulationBody te23317_2 = new SimulationBody();
		te23317_2.addFixture(te23317_2_shape);
		te23317_2.setMass(MassType.INFINITE);
		te23317_2.translate(1.75, 3.94 / 2 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23317_2);

		// TE-23318 (hinge lintel)
		Rectangle te23318_shape = new Rectangle(5.0, 1.5);
		SimulationBody te23318 = new SimulationBody();
		te23318.addFixture(te23318_shape);
		te23318.setMass(MassType.INFINITE);
		te23318.translate(0, 1.5 / 2 + 3.95 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23318);

		// TE-23316 (hinge support)
		Rectangle te23316_shape = new Rectangle(3.5, 5.44);
		SimulationBody te23316 = new SimulationBody();
		te23316.addFixture(te23316_shape);
		te23316.setMass(MassType.INFINITE);
		te23316.translate(2.5 + 0.75 + 3.5 / 2, 5.44 / 2 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23316);

		// TE-23311 (fixed hinge core)
		Rectangle te23311_shape = new Rectangle(6.63, 1.0);
		SimulationBody te23311 = new SimulationBody();
		te23311.addFixture(te23311_shape);
		te23311.setMass(MassType.INFINITE);
		te23311.translate(2.5 + 0.75 + 3.5 - 6.63 / 2, 1.0 / 2 + 5.44 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23311);

		// TE-23310 (hinge strap)
		Rectangle te23310_shape = new Rectangle(6.0, 1.0);
		SimulationBody te23310 = new SimulationBody();
		te23310.addFixture(te23310_shape);
		te23310.setMass(MassType.NORMAL); // TODO: density?
		te23310.translate(2.5 + 0.75 + 3.5 - 4.13 - 5.25 / 2, 1.0 / 2 + 5.44 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23310);

		// fixed pin
		RevoluteJoint<SimulationBody> fixedPin = new RevoluteJoint<SimulationBody>(te23311, te23310,
				new Vector2(2.5 + 0.75 + 3.5 - 4.13, 1.0 / 2 + 5.44 + 0.47));
		this.world.addJoint(fixedPin);

		// TE-23311_2 (moving hinge core)
		Rectangle te23311_2_shape = new Rectangle(6.63, 1.0);
		SimulationBody te23311_2 = new SimulationBody();
		te23311_2.addFixture(te23311_2_shape);
		te23311_2.setMass(MassType.NORMAL);
		te23311_2.translate(2.5 + 0.75 + 3.5 - 4.13 - 5.25 - 4.13 + 6.63 / 2, 1.0 / 2 + 5.44 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23311_2);

		// moving pin
		RevoluteJoint<SimulationBody> movingPin = new RevoluteJoint<SimulationBody>(te23310, te23311_2,
				new Vector2(2.5 + 0.75 + 3.5 - 4.13 - 5.25, 1.0 / 2 + 5.44 + 0.47));
		this.world.addJoint(movingPin);

		// TE-23316_2 (moving hinge support)
		Rectangle te23316_2_shape = new Rectangle(3.5, 1.5);
		SimulationBody te23316_2 = new SimulationBody();
		te23316_2.addFixture(te23316_2_shape);
		te23316_2.setMass(MassType.NORMAL);
		te23316_2.translate(2.5 + 0.75 + 3.5 - 4.13 - 5.25 - 4.13 + 3.5 / 2, 1.5 / 2 + 1.0 + 5.44 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23316_2);

		WeldJoint<SimulationBody> movingHingeSupportToMovingHingeCore = new WeldJoint<SimulationBody>(te23316_2,
				te23311_2,
				new Vector2(2.5 + 0.75 + 3.5 - 4.13 - 5.25 - 4.13 + 3.5 / 2, 1.5  + 1.0 + 5.44 + 0.47));
				this.world.addJoint(movingHingeSupportToMovingHingeCore);

						// TE-23302 (top)
	    // 22.25 + 1.5
		Rectangle te23302_shape = new Rectangle(48, 0.72);
		SimulationBody te23302 = new SimulationBody();
		te23302.addFixture(te23302_shape);
		te23302.setMass(MassType.NORMAL);
		te23302.translate( 0, 0.72/2 + 1.5 + 1.0 + 5.44 + 0.47);
		// rectangle.getLinearVelocity().set(-5.0, 0.0);
		this.world.addBody(te23302);

		WeldJoint<SimulationBody> topToMovingHingeSupport = new WeldJoint<SimulationBody>(te23302,
		te23316_2,
		new Vector2(3.5/2-1.5-3.5/2, 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addJoint(topToMovingHingeSupport);

		// // create a triangle object
		// Triangle triShape = new Triangle(
		// new Vector2(0.0, 0.5),
		// new Vector2(-0.5, -0.5),
		// new Vector2(0.5, -0.5));
		// SimulationBody triangle = new SimulationBody();
		// triangle.addFixture(triShape);
		// triangle.setMass(MassType.NORMAL);
		// triangle.translate(-1.0, 2.0);
		// // test having a velocity
		// triangle.getLinearVelocity().set(5.0, 0.0);
		// this.world.addBody(triangle);

		// create a circle
		Circle cirShape = new Circle(0.5);
		SimulationBody circle = new SimulationBody();
		circle.addFixture(cirShape);
		circle.setMass(MassType.NORMAL);
		circle.translate(2.0, 10.0);
		// test adding some force
		// circle.applyForce(new Vector2(-100.0, 0.0));
		// set some linear damping to simulate rolling friction
		circle.setLinearDamping(0.05);
		this.world.addBody(circle);

		// // try a polygon with lots of vertices
		// Polygon polyShape = Geometry.createUnitCirclePolygon(10, 1.0);
		// SimulationBody polygon = new SimulationBody();
		// polygon.addFixture(polyShape);
		// polygon.setMass(MassType.NORMAL);
		// polygon.translate(-2.5, 2.0);
		// // set the angular velocity
		// polygon.setAngularVelocity(Math.toRadians(-20.0));
		// this.world.addBody(polygon);

		// // try a compound object
		// Circle c1 = new Circle(0.5);
		// BodyFixture c1Fixture = new BodyFixture(c1);
		// c1Fixture.setDensity(0.5);
		// Circle c2 = new Circle(0.5);
		// BodyFixture c2Fixture = new BodyFixture(c2);
		// c2Fixture.setDensity(0.5);
		// Rectangle rm = new Rectangle(2.0, 1.0);
		// // translate the circles in local coordinates
		// c1.translate(-1.0, 0.0);
		// c2.translate(1.0, 0.0);
		// SimulationBody capsule = new SimulationBody();
		// capsule.addFixture(c1Fixture);
		// capsule.addFixture(c2Fixture);
		// capsule.addFixture(rm);
		// capsule.setMass(MassType.NORMAL);
		// capsule.translate(0.0, 4.0);
		// this.world.addBody(capsule);

		// SimulationBody issTri = new SimulationBody();
		// issTri.addFixture(Geometry.createIsoscelesTriangle(1.0, 3.0));
		// issTri.setMass(MassType.NORMAL);
		// issTri.translate(2.0, 3.0);
		// this.world.addBody(issTri);

		// SimulationBody equTri = new SimulationBody();
		// equTri.addFixture(Geometry.createEquilateralTriangle(2.0));
		// equTri.setMass(MassType.NORMAL);
		// equTri.translate(3.0, 3.0);
		// this.world.addBody(equTri);

		// SimulationBody rightTri = new SimulationBody();
		// rightTri.addFixture(Geometry.createRightTriangle(2.0, 1.0));
		// rightTri.setMass(MassType.NORMAL);
		// rightTri.translate(4.0, 3.0);
		// this.world.addBody(rightTri);

		// SimulationBody cap = new SimulationBody();
		// cap.addFixture(new Capsule(1.0, 0.5));
		// cap.setMass(MassType.NORMAL);
		// cap.translate(-3.0, 3.0);
		// this.world.addBody(cap);

		// SimulationBody slice = new SimulationBody();
		// slice.addFixture(new Slice(0.5, Math.toRadians(120)));
		// slice.setMass(MassType.NORMAL);
		// slice.translate(-3.0, 3.0);
		// this.world.addBody(slice);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.dyn4j.samples.framework.SimulationFrame#initializeCamera(org.dyn4j.
	 * samples.framework.Camera)
	 */
	@Override
	protected void initializeCamera(Camera camera) {
		super.initializeCamera(camera);
		camera.scale = 45.0;
	}

	/**
	 * Entry point for the example application.
	 * 
	 * @param args command line arguments
	 */
	public static void main(String[] args) {
		ChargeStation simulation = new ChargeStation();
		simulation.run();
	}
}
