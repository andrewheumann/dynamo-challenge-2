using Elements;
using Elements.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace DynamoChallenge2
{
    public static class DynamoChallenge2
    {
        /// <summary>
        /// The DynamoChallenge2 function.
        /// </summary>
        /// <param name="model">The input model.</param>
        /// <param name="input">The arguments to the execution.</param>
        /// <returns>A DynamoChallenge2Outputs instance containing computed results and the model with any new elements.</returns>
        public static DynamoChallenge2Outputs Execute(Dictionary<string, Model> inputModels, DynamoChallenge2Inputs input)
        {
            var output = new DynamoChallenge2Outputs();
            var model = output.Model;
            var baseTriangle = Polygon.Ngon(3, 1.0);

            var bottomFullEdge = baseTriangle.Segments().Last();
            var A = bottomFullEdge.PointAt(1.0);

            var bottomEdge = new Line(A, bottomFullEdge.PointAt(0.5));
            var topEdge = new Line(A, (0, 0));
            var angle = topEdge.Direction().AngleTo(bottomEdge.Direction());
            var halfAngle = angle / 2;
            var rotation = new Transform().RotatedAboutPoint(A, (0, 0, 1), halfAngle);
            var midEdge = topEdge.TransformedLine(rotation);

            var vertical = new Line(bottomEdge.End, (0, 0));
            var outsideEdges = new[] { topEdge, bottomEdge };
            var height = 0.05;

            var white = new Material("White")
            {
                Color = (0.8, 0.8, 0.8),
                SpecularFactor = 0.3,
                GlossinessFactor = 0,
                DoubleSided = true,
            };

            var parameters = new[] {
              input.ShapeParameters.A,
              input.ShapeParameters.B,
              input.ShapeParameters.C,
              input.ShapeParameters.D,
              input.ShapeParameters.E,
              input.ShapeParameters.F,
              input.ShapeParameters.G,
            };
            var sum = parameters.Sum();
            if (sum > 0.8)
            {
                parameters = parameters.Select(x => x * 0.8 / sum).ToArray();
            }
            var parameterProgressiveSum = new List<double>();
            foreach (var parameter in parameters)
            {
                parameterProgressiveSum.Add((parameterProgressiveSum.Count() == 0 ? 0.0 : parameterProgressiveSum.Last()) + parameter);
            }

            var meshes = new Dictionary<Vector3, Mesh>();
            foreach (var edge in outsideEdges)
            {
                var inwardsVector = (midEdge.PointAt(0.5) - edge.PointAt(0.5)).Unitized();
                // set up high + low points. high points are lower-case vars
                var edgeVector = edge.Direction();

                var B1 = A + edgeVector * parameterProgressiveSum[0];
                var b1 = A + edgeVector * parameterProgressiveSum[1];
                var C1 = A + edgeVector * parameterProgressiveSum[2];
                var D1 = A + edgeVector * parameterProgressiveSum[3] + inwardsVector * input.ShapeParameters.H;
                var c1 = A + edgeVector * parameterProgressiveSum[4];
                var E1 = A + edgeVector * parameterProgressiveSum[5] + inwardsVector * input.ShapeParameters.H;
                var F1 = A + edgeVector * parameterProgressiveSum[6];
                var d1 = edge.PointAt(1.0);

                var midVector = midEdge.Direction();

                var b2 = midVector * 0.242133 + A;
                var B2 = midVector * 0.3285 + A;
                var c2 = midVector * 0.4352 + A;
                var C2 = midVector * 0.589567 + A;
                var d2 = midVector * 0.7208 + A;
                midEdge.Intersects(vertical, out var D2, true);

                var transforms = new[] {
                    new Transform(),
                    new Transform().Scaled((1, -1, 1))
                };

                new List<(string name, Vector3 peak, Vector3[] points)> {
                    ("b2", b2, new [] {A, B1, B2}),
                    ("b1", b1, new [] {B1, B2, C1}),
                    ("c2", c2, new [] {B2, C1, D1, C2}),
                    ("c1", c1, new [] {C1, D1, C2, E1, F1}),
                    ("d2", d2, new [] {C2, E1, F1, D2}),
                    ("d1", d1, new [] {F1, D2})
                }.ToList().ForEach((g) =>
                {
                    for (int i = 0; i < transforms.Length; i++)
                    {
                        var xform = transforms[i];
                        var peakLoc = xform.OfPoint(g.peak) + (0, 0, height);
                        if (!meshes.TryGetValue(peakLoc, out Mesh m))
                        {
                            m = new Mesh();
                            meshes.Add(peakLoc, m);
                        }
                        var peakV = m.AddVertex(peakLoc);
                        var pl = new Polyline(g.points).TransformedPolyline(xform);
                        if (i == 1)
                        {
                            pl = pl.Reversed();
                        }
                        foreach (var seg in pl.Segments())
                        {
                            var v1 = m.AddVertex(seg.Start);
                            var v2 = m.AddVertex(seg.End);
                            m.AddTriangle(v1, v2, peakV);
                            var pgon = new Polygon(seg.Start, seg.End, peakLoc);
                        }
                        m.ComputeNormals();
                    }
                });
            }

            var center = baseTriangle.Segments().First().PointAt(1);
            var hexRotations = new[] { 0, 1, 2, 3, 4, 5 }.Select(i => new Transform().RotatedAboutPoint(center, (0, 0, 1), 360 * i / 6)).ToArray();

            var rectangle = new Polygon(new[] {
                (1,0),
                hexRotations[2].OfPoint((1,0)),
                hexRotations[3].OfPoint((1,0)),
                hexRotations[5].OfPoint((1,0)),
            });
            var offset = rectangle.Offset(0.01);
            var offsetRectangle = offset.First();

            var translationXforms = new[] {
                new Transform().Moved(hexRotations[2].OfPoint((1,0)) - (1,0)),
                new Transform(),
                new Transform().Moved((1,0) - hexRotations[2].OfPoint((1,0))),
            };

            var rotationXforms = new[] {
                  new Transform(),
                  new Transform().Rotated((0,0,1), 360 / 3),
                  new Transform().Rotated((0,0,1), 360 * 2 / 3),
              };

            var meshGroups = new Dictionary<Vector3, List<ElementInstance>>();

            foreach (var kvp in meshes)
            {
                var meshElement = new MeshElement(kvp.Value, white)
                {
                    IsElementDefinition = true
                };
                foreach (var tx in translationXforms)
                {
                    foreach (var hx in hexRotations)
                    {
                        foreach (var rx in rotationXforms)
                        {
                            var x = rx.Concatenated(hx).Concatenated(tx);
                            var key = kvp.Key;
                            var newKey = x.OfPoint(key);
                            if (!offsetRectangle.Contains((newKey.X, newKey.Y)))
                            {
                                continue;
                            }
                            if (!meshGroups.TryGetValue(newKey, out List<ElementInstance> instances))
                            {
                                instances = new List<ElementInstance>();
                                meshGroups.Add(newKey, instances);
                            }
                            instances.Add(meshElement.CreateInstance(x, null));
                        }
                    }
                }
            }

            foreach (var mg in meshGroups)
            {
                model.AddElements(mg.Value);
            }
            return output;
        }
    }
}