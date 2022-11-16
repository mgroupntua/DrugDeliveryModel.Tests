using System;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Constitutive;
using MGroup.MSolve.DataStructures;

namespace MGroup.DrugDeliveryModel.Tests.Materials
{
    /// <summary>
    /// Deformation Gradient based implementation of a Mooney Rivlin hyperelastic material and a
    /// NeoHookian hyperelastic material for
    /// Authors: Gerasimos Sotiropoulos
    /// </summary>
    public class NeoHookeanMaterial3dJ3Isochoric : IContinuumMaterial3DDefGrad
    {
        private readonly GenericConstitutiveLawState currentState;
        private double[] stresses = new double[6];
        private Matrix constitutiveMatrix = null;

        public double mi { get; set; }
        public double kappa { get; set; }

        //public LogarithmicNeoHookeanMaterial3d(double mi, double kappa)
        //{
        //	this.mi = mi;
        //	lamda = (3 * kappa - mi) / 2;
        //}

        public NeoHookeanMaterial3dJ3Isochoric(double mi, double kappa)
        {
            this.mi = mi;
            this.kappa = kappa;
        }

        public NeoHookeanMaterial3dJ3Isochoric() { }



        #region IConstitutiveLaw Members
        /// <summary>
        /// Returns the constitutive matrix of the material for the current strain state
        /// </summary>
        public IMatrixView ConstitutiveMatrix
        {
            get
            {
                if (constitutiveMatrix == null) UpdateConstitutiveMatrixAndEvaluateResponse(new double[9] { 1, 1, 1, 0, 0, 0, 0, 0, 0 });
                return constitutiveMatrix;
            }
        }

        /// <summary>
        /// Updates the material state for a given new strain point.
        /// </summary>
        /// <param name="Delta">The given strain point.</param>
        public double[] UpdateConstitutiveMatrixAndEvaluateResponse(double[] DefGradVec)
        {
            var DG = Matrix.CreateFromArray(new double[3, 3] { { DefGradVec [0], DefGradVec[3], DefGradVec[6] },
                                                { DefGradVec [7], DefGradVec[1], DefGradVec[4] },
                                                { DefGradVec [5], DefGradVec[8], DefGradVec[2] }});

            var rCG = DG.MultiplyRight(DG, true, false);

            var rCGvec = Vector.CreateFromArray(new double[] { rCG[0, 0], rCG[1, 1], rCG[2, 2], rCG[0, 1], rCG[1, 2], rCG[2, 0] });
            var rCG_inv = rCG.Invert();
            //var rCG_inv_Vec = Vector.CreateFromArray(new double[] { rCG_inv[0, 0], rCG_inv[1, 1], rCG_inv[2, 2], rCG_inv[0, 1], rCG_inv[1, 2], rCG_inv[2, 0] });

            var I_1 = rCG[0, 0] + rCG[1, 1] + rCG[2, 2];
            var vec_12 = new double[3]; var vec_22 = new double[3];
            rCGvec.CopyToArray(0, vec_12, 0, 3); rCGvec.CopyToArray(3, vec_22, 0, 3);
            //var Vec_12 = Vector.CreateFromArray(vec_12); var Vec_22 = Vector.CreateFromArray(vec_22);
            //var I_2 = 0.5 *( Math.Pow(I_1, 2) - Vec_12 * Vec_12 - 2 * (Vec_22 * Vec_22));
            var I_3 = rCG.CalcDeterminant();
            var J_3 = DG.CalcDeterminant();

            var I_1_st = Matrix.CreateIdentity(3); I_1_st.ScaleIntoThis(2);
            var I_2_st = Matrix.CreateIdentity(3); I_2_st.ScaleIntoThis(I_1); I_2_st.SubtractIntoThis(rCG); I_2_st.ScaleIntoThis(2);
            var I_3_st = rCG_inv.Scale(2 * I_3);
            var I_1_st_vec = Vector.CreateFromArray(new double[] { 2, 2, 2, 0, 0, 0 });
            //var I_2_st_vec = Vector.CreateFromArray(new double[] { I_2_st[0, 0], I_2_st[1, 1], I_2_st[2, 2], I_2_st[0, 1], I_2_st[1, 2], I_2_st[2, 0] });
            var I_3_st_vec = Vector.CreateFromArray(new double[] { I_3_st[0, 0], I_3_st[1, 1], I_3_st[2, 2], I_3_st[0, 1], I_3_st[1, 2], I_3_st[2, 0] });

            //var J_1_st = Math.Pow(I_3, (-(double)1 / 3)) * I_1_st - I_3_st.Scale(((double)1 / 3) * I_1 * Math.Pow(I_3, -((double)4 / 3)));
            //var J_2_st = Math.Pow(I_3, (-(double)2 / 3)) * I_2_st - I_3_st.Scale(((double)2 / 3) * I_2 * Math.Pow(I_3, -((double)5 / 3)));
            var J_3_st = 0.5 * Math.Pow(I_3, -(double)1 / 2) * I_3_st;
            var J_3_st_vec = Vector.CreateFromArray(new double[] { J_3_st[0, 0], J_3_st[1, 1], J_3_st[2, 2], J_3_st[0, 1], J_3_st[1, 2], J_3_st[2, 0] });


            //var I_2_stst = Matrix.CreateFromArray(new double[6, 6] { {0,4,4,0,0,0},{ 4, 0, 4, 0, 0, 0 },{ 4, 4, 0, 0, 0, 0 },
            //                                                {0,0,0,-2,0,0 },{0,0,0,0,-2,0 },{0,0,0,0,0,-2 } });
            var I_3_stst = Matrix.CreateFromArray(new double[6, 6]{ { 0, 4 * rCG[2, 2], 4 * rCG[1, 1], 0, -4 * rCG[1, 2], 0 },
                                                                    { 4 * rCG[2, 2],0,4 * rCG[0, 0],0,0,-4 * rCG[2, 0] },
                                                                    { 4 * rCG[1, 1],4 * rCG[0, 0],0,-4 * rCG[0, 1],0,0 },
                                                                { 0,0,-4 * rCG[0, 1],-2 * rCG[2, 2],2 * rCG[2, 0],2 * rCG[1, 2] },
                                                                { -4 * rCG[1, 2],0,0,2 * rCG[2, 0],-2 * rCG[0, 0],2 * rCG[0, 1] },
                                                                { 0,-4 * rCG[2, 0],0,2 * rCG[1, 2],2 * rCG[0, 1],-2 * rCG[1, 1] }});

            //var J_1_stst = (-(double)1 / 3) * Math.Pow(I_3, -((double)4 / 3)) *( (I_1_st_vec.TensorProduct(I_3_st_vec)) +
            //    I_3_st_vec.TensorProduct(I_1_st_vec) +
            //    I_1 * I_3_stst) +
            //    ((double)4 / 9) * I_1 * Math.Pow(I_3, -(double)7 / 3) * (I_3_st_vec.TensorProduct(I_3_st_vec));
            //var J_2_stst = Math.Pow(I_3, -(double)2 / 3) * I_2_stst -
            //    ((double)2 / 3) *(Math.Pow(I_3,-(double)5/3))*(I_2_st_vec.TensorProduct(I_3_st_vec) + I_3_st_vec.TensorProduct(I_2_st_vec) + I_2 * I_3_stst) +
            //    ((double)10 / 9) * I_2 * Math.Pow(I_3, -(double)8 / 3) * (I_3_st_vec.TensorProduct(I_3_st_vec));
            var J_3_stst = -(double)1 / 4 * Math.Pow(I_3, -(double)3 / 2) * I_3_st_vec.TensorProduct(I_3_st_vec) +
                (double)1 / 2 * Math.Pow(I_3, -(double)1 / 2) * I_3_stst;



            //var SpkNH = (mi * 0.5) * I_1_st - ( mi ) * ((double)1 / J_3) * ( J_3_st) + (0.5) * lamda*2*Math.Log(J_3)* ((double)1 / J_3) * J_3_st;
            //var SpkNH = (mi * 0.5) * I_1_st + kappa * (J_3 - 1) * (J_3_st);
            //SpkNH = lamda * Math.Log(J_3) * ((double)1 / J_3) * J_3_st;
            var SpkNH = 0.5 * mi * Math.Pow(J_3, -(double)2 / 3) * I_1_st
            - (double)1 / 3 * mi * I_1 * Math.Pow(J_3, -(double)5 / 3) * J_3_st
                + kappa * (J_3 - 1) * J_3_st;

            var Spk_vec = new double[] { SpkNH[0, 0], SpkNH[1, 1], SpkNH[2, 2], SpkNH[0, 1], SpkNH[1, 2], SpkNH[2, 0] };

            var Cons = new double[6, 6];
            for (int i1 = 0; i1 < 6; i1++)
            {
                for (int i2 = 0; i2 < 6; i2++)
                {
                    Cons[i1, i2] = 0.5 * mi * (-(double)2 / 3) * Math.Pow(J_3, -(double)5 / 3) * I_1_st_vec[i1] * J_3_st_vec[i2]

                        + -(double)1 / 3 * mi * Math.Pow(J_3, -(double)5 / 3) * J_3_st_vec[i1] * I_1_st_vec[i2]
                        + (double)1 / 3 * mi * I_1 * ((double)5 / 3) * Math.Pow(J_3, -(double)8 / 3) * J_3_st_vec[i1] * J_3_st_vec[i2]
                        + -(double)1 / 3 * mi * I_1 * Math.Pow(J_3, -(double)5 / 3) * J_3_stst[i1, i2]

                    + kappa * J_3_st_vec[i1] * J_3_st_vec[i2]
                    + kappa * (J_3 - 1) * J_3_stst[i1, i2];


                    //Cons[i1, i2] = +lamda * Math.Log(J_3) * ((double)1 / J_3) * J_3_stst[i1, i2] +
                    //	lamda * Math.Pow(J_3, -2) * J_3_st_vec[i1] * J_3_st_vec[i2] +
                    //	-lamda * Math.Log(J_3) * Math.Pow(J_3, -2) * J_3_st_vec[i1] * J_3_st_vec[i2];
                    //+ lamda * Math.Log(J_3) * ((double)1 / J_3) * J_3_stst[i1, i2] +
                    //-lamda * Math.Pow(J_3, -2) * J_3_st_vec[i1] * J_3_st_vec[i2] +
                    //-lamda * Math.Log(J_3) * Math.Pow(J_3, -2) * J_3_st_vec[i1] * J_3_st_vec[i2];

                }
            }

            //var Cons = new double[6, 6];
            //for (int i1 = 0; i1 < 6; i1++)
            //{
            //	for (int i2 = 0; i2 < 6; i2++)
            //	{
            //		Cons[i1, i2] = ((-(2 * mi + lamda) * 0.5 * ((double)1 / I_3) + 0.5 * lamda) * 0.5) * I_3_stst[i1, i2] - (2 * (2 * mi + lamda) * 0.5 * (-1) * Math.Pow(I_3, -2) * 0.25) * I_3_st_vec[i1] * I_3_st_vec[i2];
            //	}
            //}


            //var Spk = C1 * J_1_st + C2 * J_2_st + k_cons * (J_3-1)*J_3_st;//einai to spk_pavla
            //var Spk_vec = new double[] { Spk[0, 0], Spk[1, 1], Spk[2, 2], Spk[0, 1], Spk[1, 2], Spk[2, 0] };
            //einai to Cklrs_pavla_vec
            //var Cons = C1 * J_1_stst + C2 * J_2_stst + k_cons * (J_3_st_vec.TensorProduct(J_3_st_vec) + (J_3 - 1) * J_3_stst);

            stresses = Spk_vec.Copy();
            constitutiveMatrix = Matrix.CreateFromArray(Cons);
            //Console.WriteLine(constitutiveMatrix.CalcDeterminant());
            return stresses;
        }
        #endregion

        #region IConstitutiveLawWithGenericState Members
        /// <summary>
        /// Saves the current stress strain state of the material (after convergence of the iterative solution process
        /// for a given loading step).
        /// </summary>
        public GenericConstitutiveLawState CreateState() => currentState;
        IHaveState ICreateState.CreateState() => CreateState();
        public GenericConstitutiveLawState CurrentState
        {
            get => currentState;
            set { }
        }
        #endregion

        #region ICloneable Members
        /// <summary>
        /// Creates a clone of material object with the same parameters.
        /// </summary>
        /// <returns>The created material clone</returns>
        public object Clone() => new NeoHookeanMaterial3dJ3Isochoric() { mi = mi, kappa = kappa };

        /// <summary>
        /// Creates a clone of material object with the same parameters.
        /// </summary>
        /// <returns>The created material clone</returns>
        object ICloneable.Clone() => Clone();

        #endregion

    }
}
