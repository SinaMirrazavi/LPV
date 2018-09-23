/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Later modified by Nadia Figueroa
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "lpvDS.h"

lpvDS::lpvDS(const char  *path_dims):K_(0),M_(0) {

    /* Declare the number of the components
     * and the dimension  of the state  */
    MatrixXd fMatrix(1,1);fMatrix.setZero();

    if (fileUtils_.is_file_exist(path_dims))
        fMatrix=fileUtils_.readMatrix(path_dims);
    else{
        cout<<"The provided path for Dimensions does not exist:"<< path_dims << endl;
        ERROR();
    }
    if ((fMatrix.rows()!=2)){
        cout<<"Initialization of the Dimensions is wrong."<<endl;
        ERROR();
    }

    K_ = (int)fMatrix.coeff(0,0);
    M_ = (int)fMatrix.coeff(1,0);
    initialize_params();
}


lpvDS::lpvDS(const char  *path_dims, const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_, const char  *path_A_):K_(0),M_(0) {

    /* Declare the number of the components
     * and the dimension  of the state  */
    MatrixXd fMatrix(1,1);fMatrix.setZero();

    if ( fileUtils_.is_file_exist(path_dims))
        fMatrix= fileUtils_.readMatrix(path_dims);
    else{
        cout<<"The provided path for Dimensions does not exist:"<< path_dims << endl;
        ERROR();
    }
    if ((fMatrix.rows()!=2)){
        cout<<"Initialization of the Dimensions is wrong."<<endl;
        ERROR();
    }

    K_ = (int)fMatrix.coeff(0,0);
    M_ = (int)fMatrix.coeff(1,0);
    initialize_params();
    initialize_A(path_A_);
    initialize_gamma(path_prior_, path_mu_, path_sigma_);
}


lpvDS::~lpvDS(){

}

void lpvDS::initialize_params()
{

    /* Setup matrices */
    A_Matrix_ = new MatrixXd[K_]; for(int s=0; s<K_; s++ ){A_Matrix_[s].resize(M_,M_);}
    Prior_    = new double[K_];
    Mu_       = new VectorXd[K_]; for(int s=0; s<K_; s++ ){	Mu_[s].resize(M_);	}
    Sigma_    = new MatrixXd[K_]; for(int s=0; s<K_; s++ ){	Sigma_[s].resize(M_,M_);	}

    gamma_.resize(K_);
    gamma_.setZero();
    cout << "Initialized an M:" << M_ << " dimensional GMM-based LPV-DS with K: " << K_ << " Components" << endl;
}


void lpvDS::initialize_A(const char  *path_A_){

	/* Initialize A
     * path_A_ is the path of A matrix*/

	MatrixXd fMatrix(1,1);fMatrix.setZero();

    cout<<"** A's' **"<< endl;
    if (fileUtils_.is_file_exist(path_A_))
        fMatrix=fileUtils_.readMatrix(path_A_);
	else{
        cout<<"The provided path does not exist: "<<path_A_<<endl;
		ERROR();
	}


	if ((fMatrix.rows()!=K_*M_)||(fMatrix.cols()!=M_))
	{
        cout<<"Initialization of the A matrices is wrong!!"<<endl;
        cout<<"A_k: "<< endl << fMatrix<< endl;
        cout<<"[Proposed Dimensionality] K : "<<K_<<" M:"<<M_<<endl;
        cout<<"[Actual Dimensionality] K : "<< fMatrix.cols()/M_ <<" M:" << fMatrix.rows() << endl;
		ERROR();
	}
	int j = 0;


	for(int s=0; s<K_; s++ ){
		for(int i=0; i<M_; i++ ){
			A_Matrix_[s].row(i)=fMatrix.row(j);
			j++;
		}
	}

//    for(int s=0; s<K_; s++ ){
//        cout<<"A_Matrix["<<s<<"]"<<endl;
//        cout<<A_Matrix_[s]<<endl;
//    }

}
void lpvDS::initialize_gamma(const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_){

	/* Initialize scheduling/activation function parameters
	 *  path_prior_ is the path of the prior matrix
	 *	path_mu_ is the path of the mean matrix
	 *	path_sigma_ is the path of the covariance matrix */

	MatrixXd fMatrix;
    cout<<"** Priors **"<< endl;
    if (fileUtils_.is_file_exist(path_prior_))
	{
        fMatrix=fileUtils_.readMatrix(path_prior_);
	}
	else
	{
		cout<<"The provided path does not exist."<<endl;
		cout<<"path_prior_lpvDS "<<endl;
		cout<<path_prior_<<endl;
		ERROR();
	}
	if ((fMatrix.cols()!=K_)||(fMatrix.rows()!=1))
	{
		cout<<"Initialization of Prior is wrong."<<endl;
		cout<<"Number of components is: "<<K_<<endl;
		cout<<"Dimension of states of Prior is: "<<fMatrix.cols()<<endl;
		ERROR();
	}


	for (int i=0; i<K_; i++)
	{
		Prior_[i]=fMatrix(0,i);;
	}

//	for (int i=0; i<K_; i++)
//		cout<<Prior_[i]<<endl;


	fMatrix.setZero();
    cout<<"** Mu **"<< endl;
    if (fileUtils_.is_file_exist(path_mu_))
	{
        fMatrix=fileUtils_.readMatrix(path_mu_);
	}
	else
	{
		cout<<"The provided path does not exist."<<endl;
		cout<<"path_mu_lpvDS "<<endl;
		cout<<path_mu_<<endl;
		ERROR();
	}


	if ((fMatrix.cols()!=K_)||(fMatrix.rows()!=M_))
	{
		cout<<"Initialization of Mean is wrong."<<endl;
		cout<<"Number of components is: "<<K_<<endl;
		cout<<"Dimension of states is: "<<M_<<endl;
		cout<<"Dimension of states of Mean is: "<<fMatrix.rows()<<"*"<<fMatrix.cols()<<endl;
		ERROR();
	}

	for(int s=0; s<K_; s++ )
	{
		Mu_[s]=fMatrix.col(s);

	}

//	for(int s=0; s<K_; s++ )
//	{
//		cout<<"Mu["<<s<<"]."<<endl;
//		cout<<Mu_[s]<<endl;

//	}

	fMatrix.resize(1,1);fMatrix.setZero();
    cout<<"** Sigma **"<< endl;
    if (fileUtils_.is_file_exist(path_sigma_))
	{
        fMatrix=fileUtils_.readMatrix(path_sigma_);
	}
	else
	{
		cout<<"The provided path does not exist."<<endl;
		cout<<"path_sigma_lpvDS "<<endl;
		cout<<path_sigma_<<endl;
		ERROR();
	}

	if ((fMatrix.rows()!=K_*M_)||(fMatrix.cols()!=M_))
	{
		cout<<"Initialization of the covariance matrix is wrong."<<endl;
		cout<<"the covariance matrix : "<<endl;cout<<fMatrix<<endl;
        cout<<"Number of components is: "<< K_ <<endl;
        cout<<"Dimension of states is: "<< M_ <<endl;
		cout<<"Dimension of states of the covariance matrix is: "<<fMatrix.rows()<<"*"<<fMatrix.cols()<<endl;
		ERROR();
	}

	int s=0;

	for(int i=0; i<K_; i++ ){
		for(int j=0; j<M_; j++ ){
			Sigma_[i].row(j)=fMatrix.row(s);
			s++;
		}
	}

//	for(int i=0; i<K_; i++ ){
//		cout<<"Sigma["<<i<<"]."<<endl;
//		cout<<Sigma_[i]<<endl;
//	}

}

MatrixXd lpvDS::compute_A(VectorXd X){

	/* Calculating the matrix A */

	if ((X.rows()!=M_))
	{
		cout<<"The dimension of X in compute_A is wrong."<<endl;
		cout<<"Dimension of states is: "<<M_<<endl;
		cout<<"Dimension of X "<<X.rows()<<endl;
		ERROR();
	}

	MatrixXd A; A.resize(M_,M_);A.setZero();

	if (K_>1)
	{
		gamma_=compute_gamma(X);
	}
	else
	{
		gamma_(K_-1)=1;
	}

	for (int i=0;i<K_;i++)
	{
		A=A+A_Matrix_[i]*gamma_(i);
	}


	return A;
}

VectorXd lpvDS::compute_gamma(VectorXd X)
{
	VectorXd Theta;Theta.resize(K_);Theta.setZero();

	for (int i=0;i<K_;i++)
	{
		Theta(i)=Prior_[i]*GaussianPDF(X,Mu_[i],Sigma_[i]);
	}
	double sum=Theta.sum();
	if (sum<1e-100)
	{
		for (int i=0;i<K_;i++)
		{
			Theta(i)=1.0/K_;
		}
	}
	else
	{
		Theta=Theta/sum;
	}

	return Theta;
}

void lpvDS::ERROR()
{
	while(ros::ok())
	{

	}
}

double lpvDS::GaussianPDF(VectorXd x,VectorXd Mu,MatrixXd Sigma)
{

	double p;
	MatrixXd gfDiff;gfDiff.resize(1,M_);
	MatrixXd gfDiff_T;gfDiff_T.resize(M_,1);
	MatrixXd SigmaIIInv;SigmaIIInv.resize(M_,M_);
	double detSigmaII=0;
	MatrixXd gfDiffp;gfDiffp.resize(1,1);gfDiffp.setZero();

	detSigmaII=Sigma.determinant();
	SigmaIIInv=Sigma.inverse();
	if (detSigmaII<0)
	{
		detSigmaII=0;
	}
	gfDiff=(x - Mu).transpose();
	gfDiff_T=x - Mu;
	gfDiffp =gfDiff*SigmaIIInv* gfDiff_T;
	gfDiffp(0,0)=fabs(0.5*gfDiffp(0,0));
	p = exp(-gfDiffp(0,0)) / sqrt(pow(2.0*PI, M_)*( detSigmaII +1e-50));
	return p;
}


