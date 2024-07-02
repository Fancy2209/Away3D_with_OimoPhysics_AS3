package
{
    import away3d.entities.Mesh;
    import away3d.primitives.CapsuleGeometry;
    import away3d.primitives.CubeGeometry;
    import away3d.primitives.CylinderGeometry;
    import away3d.primitives.SphereGeometry;

    import oimo.collision.geometry.*;
    import oimo.common.*;
    import oimo.dynamics.*;
    import oimo.dynamics.constraint.joint.*;
    import oimo.dynamics.rigidbody.*;

    public class OimoUtils
    {

        private static var oimoWorld:World;

        private static var tmpVec3_0:Vec3 = new Vec3();
        private static var tmpVec3_1:Vec3 = new Vec3();

        private static var oimoStaticBodies:Vector.<RigidBody> = new <RigidBody>[];
        private static var oimoDynamicBodies:Vector.<RigidBody> = new <RigidBody>[];
        private static var oimoKinematicBodies:Vector.<RigidBody> = new <RigidBody>[];

        private static var awayStaticBodies:Vector.<Mesh> = new <Mesh>[];
        private static var awayDynamicBodies:Vector.<Mesh> = new <Mesh>[];
        private static var awayKinematicBodies:Vector.<Mesh> = new <Mesh>[];

        private static var lastTime:Number = 0;

        [inline]
        public static var radToDeg:Number = 57.295;

        public static function setWorld(w:World):void
        {
            if (oimoWorld == null)
            {
                oimoWorld = w;
            }
            else
            {
                throw "Error: World already set.";
            }
        }

        public static function updatePhysics():void
        {
            oimoWorld.step(120 / 1000);

            for (var i:int = 0; i < oimoDynamicBodies.length; i++)
            {
                var oimoBody:RigidBody = oimoDynamicBodies[i];
                var awayBody:Mesh = awayDynamicBodies[i];

                tmpVec3_0 = oimoBody.getTransform().getRotation().toEulerXyz();
                awayBody.moveTo(oimoBody.getPosition().x, oimoBody.getPosition().y, oimoBody.getPosition().z);
                awayBody.transform.appendRotation(tmpVec3_0.x * radToDeg, awayBody.rightVector, awayBody.position);
                awayBody.transform.appendRotation(tmpVec3_0.y * radToDeg, awayBody.upVector, awayBody.position);
                awayBody.transform.appendRotation(tmpVec3_0.z * radToDeg, awayBody.forwardVector, awayBody.position);
            }
        }

        [inline]
        public static function addPhysics(mesh:Mesh, type:int = 0, pos:Vector.<Number> = null, colliderType:String = "", options:Dynamic = null):RigidBody
        {
            if (pos == null)
            {
                pos = new <Number>[0, 0, 0];
            }
            tmpVec3_1.x = pos[0];
            tmpVec3_1.y = pos[1];
            tmpVec3_1.z = pos[2];

            var rBody:RigidBody = null;

            if (colliderType == "")
            {
                if (mesh.geometry is away3d.primitives.CubeGeometry)
                {
                    var cubeGeom:away3d.primitives.CubeGeometry = mesh.geometry as away3d.primitives.CubeGeometry;
                    tmpVec3_0.x = cubeGeom.width / 2;
                    tmpVec3_0.y = cubeGeom.height / 2;
                    tmpVec3_0.z = cubeGeom.depth / 2;
                    rBody = OimoUtils.addBox(OimoUtils.oimoWorld, tmpVec3_1, tmpVec3_0, type);
                }
                else if (mesh.geometry is away3d.primitives.SphereGeometry)
                {
                    var sphereGeom:away3d.primitives.SphereGeometry = mesh.geometry as away3d.primitives.SphereGeometry;
                    var sphereRadius:Number = sphereGeom.radius;
                    rBody = OimoUtils.addSphere(OimoUtils.oimoWorld, tmpVec3_1, sphereRadius, type);
                }
                else if ((mesh.geometry is away3d.primitives.CylinderGeometry))
                {
                    var cylinderGeom:away3d.primitives.CylinderGeometry = mesh.geometry as away3d.primitives.CylinderGeometry;
                    var cylinderRadius:Number = cylinderGeom.topRadius;
                    var cylinderHalfHeight:Number = cylinderGeom.height / 2;
                    rBody = OimoUtils.addCylinder(OimoUtils.oimoWorld, tmpVec3_1, cylinderRadius, cylinderHalfHeight, type);
                }
                else if ((mesh.geometry is away3d.primitives.CapsuleGeometry))
                {
                    var capsuleGeom:away3d.primitives.CapsuleGeometry = mesh.geometry as away3d.primitives.CapsuleGeometry;
                    var capsuleRadius:Number = capsuleGeom.radius;
                    var capsuleHalfHeight:Number = capsuleGeom.height / 2;
                    rBody = OimoUtils.addCapsule(OimoUtils.oimoWorld, tmpVec3_1, capsuleRadius, capsuleHalfHeight, type);
                }
            }
            else
            {

            }

            switch (type)
            {
                case RigidBodyType.STATIC:
                    OimoUtils.oimoStaticBodies.push(rBody);
                    OimoUtils.awayStaticBodies.push(mesh);

                case RigidBodyType.DYNAMIC:
                    OimoUtils.oimoDynamicBodies.push(rBody);
                    OimoUtils.awayDynamicBodies.push(mesh);

                case RigidBodyType.KINEMATIC:
                    OimoUtils.oimoKinematicBodies.push(rBody);
                    OimoUtils.awayKinematicBodies.push(mesh);
            }

            return rBody;
        }

        [inline]
        public static function getRandomInt(min:int, max:int):int
        {
            return Math.floor(Math.random() * ((max + 1) - min)) + min;
        }

        public static function getRandomFloat(min:Number, max:Number):Number
        {
            return Math.random() * (max - min) + min;
        }

        public static function addUniversalJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis1:Vec3, axis2:Vec3, sd1:SpringDamper = null, lm1:RotationalLimitMotor = null, sd2:SpringDamper = null, lm2:RotationalLimitMotor = null):UniversalJoint
        {
            var jc:UniversalJointConfig = new UniversalJointConfig();
            jc.init(rb1, rb2, anchor, axis1, axis2);
            if (sd1 != null)
                jc.springDamper1 = sd1;
            if (lm1 != null)
                jc.limitMotor1 = lm1;
            if (sd2 != null)
                jc.springDamper2 = sd2;
            if (lm2 != null)
                jc.limitMotor2 = lm2;
            var j:UniversalJoint = new UniversalJoint(jc);
            w.addJoint(j);
            return j;
        }

        public static function addGenericJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, basis1:Mat3, basis2:Mat3, translSds:Vector.<SpringDamper> = null, translLms:Vector.<TranslationalLimitMotor> = null, rotSds:Vector.<SpringDamper> = null, rotLms:Vector.<RotationalLimitMotor> = null):GenericJoint
        {
            var jc:GenericJointConfig = new GenericJointConfig();
            jc.init(rb1, rb2, anchor, basis1, basis2);
            for (var i:int = 0; i < 3; i++)
            {
                if (translSds != null && translSds[i] != null)
                    jc.translationalSpringDampers[i] = translSds[i];
                if (translLms != null && translLms[i] != null)
                    jc.translationalLimitMotors[i] = translLms[i];
                if (rotSds != null && rotSds[i] != null)
                    jc.rotationalSpringDampers[i] = rotSds[i];
                if (rotLms != null && rotLms[i] != null)
                    jc.rotationalLimitMotors[i] = rotLms[i];
            }
            var j:GenericJoint = new GenericJoint(jc);
            w.addJoint(j);
            return j;
        }

        public static function addPrismaticJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis:Vec3, sd:SpringDamper = null, lm:TranslationalLimitMotor = null):PrismaticJoint
        {
            var jc:PrismaticJointConfig = new PrismaticJointConfig();
            jc.init(rb1, rb2, anchor, axis);
            if (sd != null)
                jc.springDamper = sd;
            if (lm != null)
                jc.limitMotor = lm;
            var j:PrismaticJoint = new PrismaticJoint(jc);
            w.addJoint(j);
            return j;
        }

        public static function addRevoluteJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis:Vec3, sd:SpringDamper = null, lm:RotationalLimitMotor = null):RevoluteJoint
        {
            var jc:RevoluteJointConfig = new RevoluteJointConfig();
            jc.init(rb1, rb2, anchor, axis);
            if (sd != null)
                jc.springDamper = sd;
            if (lm != null)
                jc.limitMotor = lm;
            var j:RevoluteJoint = new RevoluteJoint(jc);
            w.addJoint(j);
            return j;
        }

        public static function addCylindricalJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis:Vec3, rotSd:SpringDamper = null, rotLm:RotationalLimitMotor = null, traSd:SpringDamper = null, traLm:TranslationalLimitMotor = null):CylindricalJoint
        {
            var jc:CylindricalJointConfig = new CylindricalJointConfig();
            jc.init(rb1, rb2, anchor, axis);
            if (rotSd != null)
                jc.rotationalSpringDamper = rotSd;
            if (rotLm != null)
                jc.rotationalLimitMotor = rotLm;
            if (traSd != null)
                jc.translationalSpringDamper = traSd;
            if (traLm != null)
                jc.translationalLimitMotor = traLm;
            var j:CylindricalJoint = new CylindricalJoint(jc);
            w.addJoint(j);
            return j;
        }

        public static function addSphericalJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3):SphericalJoint
        {
            var jc:SphericalJointConfig = new SphericalJointConfig();
            jc.init(rb1, rb2, anchor);
            var j:SphericalJoint = new SphericalJoint(jc);
            w.addJoint(j);
            return j;
        }

        public static function addSphere(w:World, center:Vec3, radius:Number, type:int):RigidBody
        {
            return addRigidBody(w, center, new oimo.collision.geometry.SphereGeometry(radius), type);
        }

        public static function addBox(w:World, center:Vec3, halfExtents:Vec3, type:int):RigidBody
        {
            return addRigidBody(w, center, new oimo.collision.geometry.BoxGeometry(halfExtents), type);
        }

        public static function addCylinder(w:World, center:Vec3, radius:Number, halfHeight:Number, type:int):RigidBody
        {
            return addRigidBody(w, center, new oimo.collision.geometry.CylinderGeometry(radius, halfHeight), type);
        }

        public static function addCone(w:World, center:Vec3, radius:Number, halfHeight:Number, type:int):RigidBody
        {
            return addRigidBody(w, center, new oimo.collision.geometry.ConeGeometry(radius, halfHeight), type);
        }

        public static function addCapsule(w:World, center:Vec3, radius:Number, halfHeight:Number, type:int):RigidBody
        {
            return addRigidBody(w, center, new oimo.collision.geometry.CapsuleGeometry(radius, halfHeight), type);
        }

        public static function addRigidBody(w:World, center:Vec3, geom:Geometry, type:int):RigidBody
        {
            var shapec:ShapeConfig = new ShapeConfig();
            shapec.geometry = geom;
            var bodyc:RigidBodyConfig = new RigidBodyConfig();
            bodyc.type = type;
            bodyc.position = center;
            var body:RigidBody = new RigidBody(bodyc);
            body.addShape(new Shape(shapec));
            w.addRigidBody(body);
            return body;
        }

    }
}