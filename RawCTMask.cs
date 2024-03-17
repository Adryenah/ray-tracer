using System;
using System.IO;
using System.Text.RegularExpressions;

namespace rt;

public class RawCtMask: Geometry
{
    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public RawCtMask(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap) : base(Color.NONE)
    {
        _position = position;
        _scale = scale;
        _colorMap = colorMap;

        var lines = File.ReadLines(datFile);
        foreach (var line in lines)
        {
            var kv = Regex.Replace(line, "[:\\t ]+", ":").Split(":");
            if (kv[0] == "Resolution")
            {
                _resolution[0] = Convert.ToInt32(kv[1]);
                _resolution[1] = Convert.ToInt32(kv[2]);
                _resolution[2] = Convert.ToInt32(kv[3]);
            } else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(_resolution[0]*_thickness[0]*scale, _resolution[1]*_thickness[1]*scale, _resolution[2]*_thickness[2]*scale);

        var len = _resolution[0] * _resolution[1] * _resolution[2];
        _data = new byte[len];
        using FileStream f = new FileStream(rawFile, FileMode.Open, FileAccess.Read);
        if (f.Read(_data, 0, len) != len)
        {
            throw new InvalidDataException($"Failed to read the {len}-byte raw data");
        }
    }
    
    private ushort Value(int x, int y, int z)
    {
        if (x < 0 || y < 0 || z < 0 || x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }

    public override Intersection GetIntersection(Line ray, double minDistance, double maxDistance)
    {
        // Check for intersection with bounding box
        var (boxIntersectionMin, boxIntersectionMax) = IntersectsRayBox(ray, _v0, _v1);

        if (boxIntersectionMin is null)
        {
            return Intersection.NONE;
        }

        // Extract values from tuple
        var intersectionMin = boxIntersectionMin.Value;
        var intersectionMax = boxIntersectionMax.Value;

        double accumulatedDistance = 0;

        // Calculate entry and exit points of the ray in the bounding box
        Vector entryPoint = ray.X0 + ray.Dx * intersectionMin;
        Vector exitPoint = ray.X0 + ray.Dx * intersectionMax;

        Vector currentPosition = entryPoint;
        Color accumulatedColor = Color.NONE;
        Vector normal = null;

        // Calculate step vector for ray traversal
        var stepVector = new Vector(ray.Dx).Normalize();

        // Calculate the number of steps based on the distance between entry and exit points
        int steps = (int)((exitPoint - entryPoint).Length() / stepVector.Length());
        var lastIndex = GetIndexes(stepVector);

        for (int i = 0; i < steps; i++)
        {
            currentPosition = currentPosition + stepVector;

            // Get indexes in the 3D space
            var indexes = GetIndexes(currentPosition);

            // Skip repeated indexes
            if (lastIndex == indexes) continue;
            lastIndex = indexes;

            // Get color at the current position
            Color color = GetColor(indexes);

            // Skip transparent colors
            if (color.Alpha == 0) continue;

            // Adjust transparency threshold
            if (accumulatedColor.Alpha < 0.01)
            {
                accumulatedDistance = ((currentPosition - ray.X0).Length() / ray.Dx.Length()) * 0.91;
            }

            // Blend colors
            accumulatedColor = Blend(accumulatedColor, color);

            // Get local normal and accumulate
            var localNormal = GetNormal(indexes);
            normal = normal is null ? localNormal : (normal + localNormal).Normalize();

            // Break if fully opaque
            if (accumulatedColor.Alpha == 1) break;
        }

        // Check accumulated transparency and return result
        if (accumulatedColor.Alpha < 0.01)
        {
            return Intersection.NONE;
        }
        else
        {
            return new Intersection(true, true, this, ray, accumulatedDistance, normal, Material.FromColor(accumulatedColor), accumulatedColor);
        }
    }

    (double? entryPoint, double? exitPoint) IntersectsRayBox(Line ray, Vector minCorner, Vector maxCorner)
    {
        // Calculate intersection values for each dimension
        double tMinX = (minCorner.X - ray.X0.X) / ray.Dx.X;
        double tMaxX = (maxCorner.X - ray.X0.X) / ray.Dx.X;

        // Swap values if needed
        if (tMinX > tMaxX) Swap(ref tMinX, ref tMaxX);

        double tMinY = (minCorner.Y - ray.X0.Y) / ray.Dx.Y;
        double tMaxY = (maxCorner.Y - ray.X0.Y) / ray.Dx.Y;

        if (tMinY > tMaxY) Swap(ref tMinY, ref tMaxY);

        // Check for non-overlapping intervals
        if ((tMinX > tMaxY) || (tMinY > tMaxX)) return (null, null);

        // Update tMin and tMax based on Y intervals
        if (tMinY > tMinX) tMinX = tMinY;
        if (tMaxY < tMaxX) tMaxX = tMaxY;

        double tMinZ = (minCorner.Z - ray.X0.Z) / ray.Dx.Z;
        double tMaxZ = (maxCorner.Z - ray.X0.Z) / ray.Dx.Z;

        if (tMinZ > tMaxZ) Swap(ref tMinZ, ref tMaxZ);

        // Check for non-overlapping intervals
        if ((tMinX > tMaxZ) || (tMinZ > tMaxX)) return (null, null);

        // Return valid intersection values
        return (tMinX, tMaxX);
    }

    void Swap<T>(ref T a, ref T b)
    {
        T temp = a;
        a = b;
        b = temp;
    }

    Color Blend(Color acumulatedColor, Color newColor)
    {
        double ComputeUpdate(double newVal, double oldVal) => (newVal * acumulatedColor.Alpha + oldVal * newColor.Alpha * (1 - acumulatedColor.Alpha));

        double a = Math.Min(acumulatedColor.Alpha + newColor.Alpha * ( 1 - acumulatedColor.Alpha ) , 1);
        double r = Math.Min(ComputeUpdate(newColor.Red, acumulatedColor.Red) / a, 1);
        double g = Math.Min(ComputeUpdate(newColor.Green, acumulatedColor.Green) / a, 1);
        double b = Math.Min(ComputeUpdate(newColor.Blue, acumulatedColor.Blue) / a, 1);
        return new Color(r, g, b, a);
    }

    private int[] GetIndexes(Vector v)
    {
        return new []{
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale), 
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)};
    }
    private Color GetColor(int[] idx)
    {
        ushort value = Value(idx[0], idx[1], idx[2]);
        return _colorMap.GetColor(value);
    }

    private Vector GetNormal(int[] idx)
    {
        double x0 = Value(idx[0] - 1, idx[1], idx[2]);
        double x1 = Value(idx[0] + 1, idx[1], idx[2]);
        double y0 = Value(idx[0], idx[1] - 1, idx[2]);
        double y1 = Value(idx[0], idx[1] + 1, idx[2]);
        double z0 = Value(idx[0], idx[1], idx[2] - 1);
        double z1 = Value(idx[0], idx[1], idx[2] + 1);

        return new Vector(x1 - x0, y1 - y0, z1 - z0).Normalize();
    }
}