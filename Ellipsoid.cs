using System;


namespace rt
{
    public class Ellipsoid : Geometry
    {
        private Vector Center { get; }
        private Vector SemiAxesLength { get; }
        private double Radius { get; }
        
        
        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Color color) : base(color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            // Calculate vector from the center of the ellipsoid to the line's origin
            var centerToLine = line.X0 - Center;

            // Calculate squared semi-axes length
            var sqrdSAL = SemiAxesLength.Square();

            // Coefficients of the quadratic equation for intersection
            var a = line.Dx.Square().ElementWiseDivide(sqrdSAL).Sum();
            var b = (line.Dx.ElementWiseMultiply(centerToLine) * 2.0).ElementWiseDivide(sqrdSAL).Sum();
            var c = centerToLine.Square().ElementWiseDivide(sqrdSAL).Sum() - Radius * Radius;

            // Calculate the discriminant
            var delta = b * b - 4 * a * c;

            // Check for no real roots (no intersection)
            if (delta <= 0)
            {
                return new Intersection();
            }

            // Calculate intersection points
            var t1 = (-b - Math.Sqrt(delta)) / (2 * a);
            var t2 = (-b + Math.Sqrt(delta)) / (2 * a);

            // Check if t1 is within the valid range
            if (minDist < t1 && t1 < maxDist)
            {
                return new Intersection(true, true, this, line, t1, Normal(line.CoordinateToPosition(t1)), Material, Color);
            }

            // Check if t2 is within the valid range
            if (minDist < t2 && t2 < maxDist)
            {
                return new Intersection(true, true, this, line, t2, Normal(line.CoordinateToPosition(t2)), Material, Color);
            }

            // No valid intersection within the specified distance range
            return new Intersection();
        }

        private Vector Normal(Vector point)
        {
            return ((point - Center) * 2.0).ElementWiseDivide(SemiAxesLength.Square()).Normalize();
        }
    }
}
