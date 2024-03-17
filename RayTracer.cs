using System;

namespace rt
{
    class RayTracer
    {
        private Geometry[] geometries;
        private Light[] lights;

        public RayTracer(Geometry[] geometries, Light[] lights)
        {
            this.geometries = geometries;
            this.lights = lights;
        }

        private double ImageToViewPlane(int n, int imgSize, double viewPlaneSize)
        {
            return n * viewPlaneSize / imgSize - viewPlaneSize / 2;
        }

        private Intersection FindFirstIntersection(Line ray, double minDist, double maxDist)
        {
            var intersection = Intersection.NONE;

            foreach (var geometry in geometries)
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }

            return intersection;
        }

        private Intersection FindFirstIntersectionWithoutVolumterics(Line ray, double minDist, double maxDist)
        {
            var intersection = Intersection.NONE;

            var volumetrics = geometries.OfType<RawCtMask>().ToList();
            foreach (var geometry in geometries.Except(volumetrics))
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }

            return intersection;
        }

        private Color ComputeLightContribution(Light light, Intersection intersection)
        {
            if (!FindFirstIntersection(new Line(intersection.Position, light.Position), 1, 1000).Valid)
            //if (!FindFirstIntersectionWithoutVolumterics(new Line(intersection.Position, light.Position), 1, 1000).Valid)
            {
                var E = intersection.Line.Dx * -1.0;
                var N = intersection.Normal;
                var T = (light.Position - intersection.Position).Normalize();
                var R = N * (N * T) * 2 - T;

                var color = intersection.Material.Ambient * light.Ambient;
                if (N * T > 0) color += intersection.Material.Diffuse * light.Diffuse * (N * T);
                if (E * R > 0) color += intersection.Material.Specular * light.Specular * Math.Pow(E * R, intersection.Material.Shininess);
                return color * light.Intensity;
            }
            else
            {
                return new Color();
            }
        }

        public void Render(Camera camera, int width, int height, string filename)
        {
            var background = new Color(0.2, 0.2, 0.2, 1.0);

            var up = camera.Up.Normalize();
            var right = (camera.Up ^ camera.Direction).Normalize();

            var image = new Image(width, height);

            var vecW = camera.Direction * camera.ViewPlaneDistance;

            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    var colorToSet = background;
                    var pixelPosition = camera.Position + vecW + up * ImageToViewPlane(j, height, camera.ViewPlaneHeight) + right * ImageToViewPlane(i, width, camera.ViewPlaneWidth);
                    var ray = new Line(camera.Position, pixelPosition);
                    var intersection = FindFirstIntersection(ray, 0, 400);
                    if (intersection.Valid)
                    {
                        colorToSet = new Color(0,0,0,1);
                        foreach(var light in lights)
                        {
                            colorToSet += ComputeLightContribution(light, intersection);
                        }
                    }

                    image.SetPixel(i, j, colorToSet);
                }
            }

            image.Store(filename);
        }
    }
}