using Elements;
using Elements.Geometry;
using Elements.Geometry.Solids;
using Elements.Geometry.Profiles;
using System;
using System.Collections.Generic;
using System.Linq;

namespace TrussBridge
{
    public static class TrussBridge
    {
        /// <summary>
        /// A basic truss bridge.
        /// </summary>
        /// <param name="model">The input model.</param>
        /// <param name="input">The arguments to the execution.</param>
        /// <returns>A TrussBridgeOutputs instance containing computed results and the model with any new elements.</returns>
        public static TrussBridgeOutputs Execute(Dictionary<string, Model> inputModels, TrussBridgeInputs input)
        {
            var model = new Model();

            var lightConcrete = new Material("Light Concrete", Colors.White, 0.1, 0.0);
            var elevatedPath = new Line(input.Path.Start, new Vector3(input.Path.End.X, input.Path.End.Y, input.EndElevation));
            var rep = new Representation(new List<SolidOperation>() { new Sweep(Polygon.Rectangle(new Vector3(-input.Width / 2, 0.1), new Vector3(input.Width / 2, 0.2)), elevatedPath, 0, 0, 0, false) });
            var bridgeDeck = new GeometricElement(new Transform(), lightConcrete, rep, false, Guid.NewGuid(), "Bridge Deck");
            model.AddElement(bridgeDeck);

            var totalSteelLength = 0.0;
            var primaryProfile = WideFlangeProfileServer.Instance.GetProfileByType(WideFlangeProfileType.W14x22);
            var secondaryProfile = WideFlangeProfileServer.Instance.GetProfileByType(WideFlangeProfileType.W8x48);

            // Girders
            var l = elevatedPath.Length();
            var right = elevatedPath.Offset(input.Width / 2, false);
            var upperRight = new Line(right.Start + new Vector3(0, 0, input.Height), right.End + new Vector3(0, 0, input.Height));
            var left = elevatedPath.Offset(-input.Width / 2, false);
            var upperLeft = new Line(left.Start + new Vector3(0, 0, input.Height), left.End + new Vector3(0, 0, input.Height));
            totalSteelLength += l * 4;

            model.AddElement(new Beam(right, MatchProfileWithOverride(right, primaryProfile, input.Overrides)));
            model.AddElement(new Beam(left, MatchProfileWithOverride(left, primaryProfile, input.Overrides)));
            model.AddElement(new Beam(upperRight, MatchProfileWithOverride(upperRight, primaryProfile, input.Overrides)));
            model.AddElement(new Beam(upperLeft, MatchProfileWithOverride(upperLeft, primaryProfile, input.Overrides)));

            Transform tPrev = null;

            for (var i = 0.0; i <= l; i += 3.0)
            {
                // Rectangular frames
                var u = i / l;
                var t = elevatedPath.TransformAt(u);
                t.Move(new Vector3(0, 0, input.Height / 2));
                var p = Polygon.Rectangle(input.Width, input.Height).TransformedPolygon(t);

                foreach (var s in p.Segments())
                {
                    model.AddElement(new Beam(s, MatchProfileWithOverride(s, secondaryProfile, input.Overrides)));
                    totalSteelLength += s.Length();
                }

                // Bracing
                if (i > 0)
                {
                    var a = t.OfPoint(new Vector3(input.Width / 2, input.Height / 2));
                    var b = tPrev.OfPoint(new Vector3(input.Width / 2, -input.Height / 2));
                    var braceRightCl = new Line(a, b);
                    var braceRightProfile = MatchProfileWithOverride(braceRightCl, secondaryProfile, input.Overrides);
                    var rightBrace = new Beam(braceRightCl, braceRightProfile);
                    model.AddElement(rightBrace);
                    totalSteelLength += rightBrace.Curve.Length();

                    var c = t.OfPoint(new Vector3(-input.Width / 2, input.Height / 2));
                    var d = tPrev.OfPoint(new Vector3(-input.Width / 2, -input.Height / 2));
                    var braceLeftCl = new Line(a, b);
                    var braceLeftProfile = MatchProfileWithOverride(braceRightCl, secondaryProfile, input.Overrides);
                    var leftBrace = new Beam(braceLeftCl, braceLeftProfile);
                    model.AddElement(leftBrace);
                    totalSteelLength += leftBrace.Curve.Length();
                }
                tPrev = t;
            }

            // Abutments
            var pathDir = elevatedPath.Direction();
            foreach (var location in input.AbutmentLocations)
            {
                var distance = pathDir.Dot(location - elevatedPath.Start);
                var p = elevatedPath.Start + pathDir * distance;
                var depthLine = new Line(new Vector3(p.X, p.Y, -5), p);
                model.AddElement(new ModelCurve(depthLine));
                var abutmentShape = Polygon.Rectangle(input.Width + 2, 2);
                var abutmentT = elevatedPath.TransformAt(distance / l);

                var finalT = new Transform(depthLine.Start, abutmentT.XAxis, Vector3.ZAxis);
                var mass = new Mass(abutmentShape, depthLine.Length(), lightConcrete, finalT);
                model.AddElement(mass);
            }

            var output = new TrussBridgeOutputs(input.Path.Length(), totalSteelLength);
            output.Model = model;
            return output;
        }

        private static Profile MatchProfileWithOverride(Line line, Profile defaultProfile, Overrides overrides)
        {
            var selectedProfile = defaultProfile;
            if (overrides != null)
            {
                var o = overrides.Profile.FirstOrDefault(ov =>
                {
                    var seg = (Line)ov.Identity.Curve;
                    return seg.Start.IsAlmostEqualTo(line.Start) && seg.End.IsAlmostEqualTo(line.End);
                });

                if (o != null)
                {
                    var profileName = Enum.GetName(typeof(ProfileValueProfileName), o.Value.ProfileName);
                    selectedProfile = WideFlangeProfileServer.Instance.GetProfileByName(profileName);
                }
            }
            return selectedProfile;
        }
    }
}