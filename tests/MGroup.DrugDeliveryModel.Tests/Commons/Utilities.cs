using System.Collections.Generic;
using MGroup.MSolve.DataStructures;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.MSolve.Discretization.Entities;
using Xunit;
using System;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Transient;
using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Continuum;

namespace MGroup.DrugDeliveryModel.Tests.Commons
{
	public static class Utilities
	{
		public static Model GetParallelepipedMesh()
		{
            var nnx = 3;
            var nny = 3;
            var nnz = 3;
            var lx = 1;
            var ly = 1;
            var lz = 1;
            var maxX = 1;
            var maxY = 1;
            var maxZ = 1;
            var mesh = new OrthogonalParallelepipedHexa8MeshGenerator(nnx, nny, nnz, lx, ly, lz, maxX, maxY, maxZ);
            var nodes = mesh.Nodes;

            //var mesh = new ComsolMeshReader(fileName);
            //var nodes = mesh.NodesDictionary;

            var model = new Model();
            model.SubdomainsDictionary[0] = new Subdomain(id: 0);

            foreach (var node in nodes.Values)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

            var elasticMaterial1 = new ElasticMaterial3D(youngModulus: 1, poissonRatio: 0.3);
            var elementFactory = new ContinuumElement3DFactory(elasticMaterial1, new TransientAnalysisProperties(1, 0, 0));

            //var domains = new Dictionary<int, double[]>(2);
            foreach (var elementConnectivity in mesh.ElementConnectivity)
            {
                var domainId = elementConnectivity.Value.Item3;
                var element = elementFactory.CreateElement(elementConnectivity.Value.Item1, elementConnectivity.Value.Item2);
                model.ElementsDictionary.Add(elementConnectivity.Key, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
            }

            var faceXYNodes = new List<INode>();
            var faceXZNodes = new List<INode>();
            var faceYZNodes = new List<INode>();

            foreach (var node in model.NodesDictionary.Values)
            {
                if (Math.Abs(0 - node.Z) < 1E-9) faceXYNodes.Add(node);
                if (Math.Abs(0 - node.Y) < 1E-9) faceXZNodes.Add(node);
                if (Math.Abs(0 - node.X) < 1E-9) faceYZNodes.Add(node);
            }

            var constraints = new List<INodalDisplacementBoundaryCondition>();
            foreach (var node in faceXYNodes)
            {
                constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationZ, amount: 0d));
            }
            foreach (var node in faceXZNodes)
            {
                constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationY, amount: 0d));
            }
            foreach (var node in faceYZNodes)
            {
                constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationX, amount: 0d));
            }

            INode maxDistanceNode = null;
            double currentMaxDistance = 0;
            foreach (INode node in model.NodesDictionary.Values)
            {
                double distance = Math.Sqrt(Math.Pow(node.X, 2) + Math.Pow(node.Y, 2) + Math.Pow(node.Z, 2));
                if (distance > currentMaxDistance)
                {
                    currentMaxDistance = distance;
                    maxDistanceNode = node;
                }
            }

            var loads = new List<INodalLoadBoundaryCondition>();

            loads.Add(new NodalLoad
            (
                maxDistanceNode,
                StructuralDof.TranslationZ,
                amount: 0.00000001
            ));

            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

            return model;
        }

		public static bool AreDisplacementsSame(IReadOnlyList<double[]> expectedDisplacements,
			TotalDisplacementsPerIterationLog computedDisplacements, double tolerance)
		{
			var comparer = new ValueComparer(tolerance);
			for (var iter = 0; iter < expectedDisplacements.Count; ++iter)
			{
				for (var i = 0; i < expectedDisplacements[iter].Length; ++i)
				{
					var expected = expectedDisplacements[iter][i];
					(var node, var dof) = computedDisplacements.WatchDofs[i];
					var computed = computedDisplacements.GetTotalDisplacement(iter, node, dof);

					if (!comparer.AreEqual(expected, computed)) return false;
				}
			}
			return true;
		}

		public static bool AreDisplacementsSame(IReadOnlyList<double[]> expectedDisplacements, IncrementalDisplacementsLog computedDisplacements, double tolerance)
		{
			var comparer = new ValueComparer(tolerance);
			for (var iter = 0; iter < expectedDisplacements.Count; ++iter)
			{
				for (var i = 0; i < expectedDisplacements[iter].Length; ++i)
				{
					var expected = expectedDisplacements[iter][i];
					(var node, var dof) = computedDisplacements.WatchDofs[i];
					var computed = computedDisplacements.GetTotalDisplacement(iter, node, dof);

					if (!comparer.AreEqual(expected, computed)) return false;
				}
			}
			return true;
		}

		public static bool AreDisplacementsSame(double[] expectedDisplacements, double[] computedDisplacements, double tolerance)
		{
			var comparer = new ValueComparer(tolerance);

			for (var i = 0; i < expectedDisplacements.Length; i++)
			{
				if (!comparer.AreEqual(expectedDisplacements[i], computedDisplacements[i])) return false;
			}

			return true;
		}

		public static void CheckModelSubdomains(Dictionary<int, int[]> expectedSubdomains, Model model)
		{
			for (var i = 0; i < expectedSubdomains.Count; i++)
			{
				var subdomainElements = model.SubdomainsDictionary[i].Elements;
				Assert.Equal(expectedSubdomains[i].Length, model.SubdomainsDictionary[i].Elements.Count);
				for (var j = 0; j < expectedSubdomains[i].Length; j++)
				{
					Assert.Equal(expectedSubdomains[i][j], subdomainElements[j].ID);
				}
			}
		}

        public static bool AreTensorsEqual(IReadOnlyList<double[]> tensors1, IReadOnlyList<double[]> tensors2, double tolerance)
        {
            if (tensors1.Count != tensors2.Count) return false;
            for (int i = 0; i < tensors1.Count; ++i)
            {
                if (tensors1[i].Length != tensors2[i].Length) return false;
                for (int j = 0; j < tensors1[i].Length; ++j)
                {
                    if (!AreValuesEqual(tensors1[i][j], tensors2[i][j], tolerance)) return false;
                }
            }
            return true;
        }

        public static bool AreValuesEqual(double value1, double value2, double tolerance)
        {
            if (Math.Abs(value2) <= tolerance) // Can't divide with expected ~= 0. 
            {
                if (Math.Abs(value1) <= tolerance) return true;
                else return false;
            }
            else return (Math.Abs(1.0 - value1 / value2) < tolerance) ? true : false;
        }
    }
}
