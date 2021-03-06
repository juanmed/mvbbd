#ifndef BB_V1202008_HPP_
#define BB_V1202008_HPP_

#include <eigen3/Eigen/Dense>
#include <list>




/** class: BBox
    *Description:This class is used to create a structure for bounding box
    *Original author: https://github.com/manuelbonilla/pacman_bbox
*/


class BBox
{
    public:
    Eigen::Matrix<double, 4, 4> T;
    Eigen::MatrixXd Points;
    Eigen::Matrix<double, 2,3> Isobox;
    double Isobox_volume;
    Eigen::Matrix<double, 4, 1> centroid;
    double distance_cm_orig;
    int num_points_;

    //CObject();
    BBox ( int num_points );
    
    BBox(){};
    
    void setNumPoints (int num_points);
    /**Function: SetPoint
        * Input: constant, variable
        * Description: initializes the points and transformation           
    */
    void SetPoints ( const Eigen::MatrixXd& EigenPoints );
    


    /**Function: SetPoint
        * Input: constant, variable
        * Description: Initializes the points with the mesh values             
    */
    void SetPoint ( int i, double x, double y, double z );
    
    /**Function: SetT
        * Input: transform
        * Description: Calculates the new transformation          
    */
    void SetT ( Eigen::Matrix<double , 4, 4>& Tnew );
   

    /** Function: doPCA
        * Inputs: Matrix, tranformation
        * Description: Finds the PCA axis. The steps to perform PCA for the purpose of visualisation are
        *        1)Translate the data so that the centre is at the origin
        *        2)Calculate the covariance matrix
        *        3)Find the principal components "using Singular Value Decomposition (SVD)"
        *        4)Reduce the data using the selected principal components
    */
    void doPCA ( const Eigen::Matrix<double, 4, 4>& Told );

    
    /**Function: box_distance
        *Inputs: first BBox, actual BBox
        *Description: Calculates distance between original object and actual boxes
    */
    void box_distance ( BBox bigestbox, BBox actual);


 
  ~BBox(){};

};


#endif /* BB_V1202008_HPP_ */
