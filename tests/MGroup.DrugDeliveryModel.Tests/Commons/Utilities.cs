using System.Collections.Generic;
using MGroup.MSolve.DataStructures;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.MSolve.Discretization.Entities;
using Xunit;
using System;

namespace MGroup.DrugDeliveryModel.Tests.Commons
{
	public static class Utilities
	{
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
