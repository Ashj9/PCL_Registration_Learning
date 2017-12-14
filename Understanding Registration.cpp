/*typedef int* int_p1;
int_p1 a, b, c;  // a, b, and c are all int pointers.

#define int_p2 int*
int_p2 a, b, c;  // only the first is a pointer!*/


#include<boost/make_shared.hpp>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/point_representation.h>

// now include the io header files

#include<pcl/io/pcd_io.h>

#include<pcl/features/normal_3d.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/filter.h>  //note that filters and single filter

//for doing registration , you need registration packages

#include<pcl/registration/icp.h>
#include<pcl/registration/icp_nl.h>
#include<pcl/registration/transforms.h>

//and finally , we have to visualize the results
#include<pcl/visualization/pcl_visualizer.h>
/*Generic field handler class for colors.
Uses an user given field to extract 1D data and display the color at each point using a min-max lookup table. */
using pcl::visualization::PointCloudColorHandlerGenericField ;
using pcl::visualization::PointCloudColorHandlerCustom ;
//some more typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCoudWithNormals;

//create global variables for visualization purpose
pcl::visualization::PCLVisualizer *p;
// its left and right view ports

int vp_1, vp_2; //vp for view port

//we will handle cloud point and file name as a pair by using following structure

struct PCD{
    PointCloud::Ptr cloud;
    std::string file_name; //nothing bur use of namespace 'std'
    PCD() : cloud (new PointCloud){};  //is it kind of constructor here?
};

//define a new point representation for <x,y,z,curvature>
//but why? and which?
class MyPointRepresentation: public pcl ::PointRepresentation <PointNormalT>
{
        using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
    public:
        MyPointRepresentation()
        {        
        //define the number of dimensions
              nr_dimensions_=4;
        }
    //override the 'copyToFloatArray()' method to define our feature vector 
     virtual void copyToFloatArray(const PointNormalT &p, float *out)  const   /*this 'const' at the last is 
     overloading the methods on the basis of 'const' type.*/
        {
        // for <x,y,z, curvature>
            out[0]=p.x;
            out[1]=p.y;
            out[2]=p.z;
            out[3]=p.curvature;
         }
};

// display of source and target on the first viewport of the visualizer

void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    // cleared screen probably
    p->removePointCloud("vp1_target"); //vp for viewport and we have two ports
    p->removeAllPointClouds("vp1_source");
    p->spin();
    PCL_INFO("Enter q to see the point clouds");
    
    //using the colour handlers
    PointCloudColorHandlerGenericField<PointT> target_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerGenericField<PointT> source_h (cloud_source, 255, 0, 0);
    
    p->addPointCloud(cloud_target, target_h, "vp1_target", vp_1);
    p->addPointCloud(cloud_source, source_h , "vp1_source", vp_1);
    
    PCL_INFO("Press q to stop seeing the current view and start point cloud registration");
    p->spin();
}

// Display source and target on the second viewport of the visualizer

void showCloudsRight(const PointCoudWithNormals ::Ptr cloud_target, const PointCoudWithNormals::Ptr cloud_source)
{

    p->removeAllPointClouds("source");
    p->removeAllPointClouds("target");
    
//using the color handlers
    PointCloudColorHandlerGenericField<PointNormalT> target_colour_handler (cloud_target, "curvature");
    if ( ! target_colour_handler.isCapable() )
    {
        PCL_WARN("Can not create curvature color handler!!");
    }
    
    PointCloudColorHandlerGenericField<PointNormalT> source_colour_handler (cloud_source, "curvature");
    if ( ! source_colour_handler.isCapable() )
    {
        PCL_WARN("Can not create curvature color handler!!");
    }
    
    //now adding the desired point cloud
    p->addPointCloud(cloud_target, target_colour_handler, "target" , vp_2 );
    p->addPointCloud(cloud_source, source_colour_handler, "source" , vp_2 );
    
    p->spinOnce();
}
 
//load  the set of PCD files that we want to register together
//param argc is the number of arguments i.e. pobably the number of PCD files
// param argv is the actual command line arguments (pass them from main() )
// param 'models' is the output argument which will be resultant vector of the PCD datasets

void loadData (int argc, char **argv, std::vector<  ( PCD , Eigen::aligned_allocator<PCD> ) > &models )
{
    std::string extention (".pcd"); //defind a string variable which is 'extention' and passed value as ".pcd"
    //lets suppose the first model is actual test model, we are skipping the program name
    for (int i=1; i<=argc; i++)
    {
      std::string file_name=std::string(argv[i]); // collecting the filenames
      //filename has to be at least 5 chars long , including 4 chars for ".pcd" extention
      if (file_name.size() <= extention.size() )
          continue; //skip that file
      
      std:: transform(file_name.begin(), file_name.end(), file_name.begin()/*here the result starts getting stored*/, 
              ( int(*) (int) ) tolower /*this is the operation*/); //probably storing the integer pointer for the lowercase letters
      //we will check if the argument is a valid PCD file
      
      if( file_name.compare(file_name.size()-extention.size() , extention.size(), extention ) == 0 )
      {
      //Load the cloud and save it into Global list of models
          PCD m; //Remember we have declared PCD as a structure. Here, 'm' for model
          m.file_name=argv[i];
          pcl::io::loadPCDFile (argv[i], *m.cloud);
          //remove the NAN points from the cloud, like kind of preprocessing point cloud data
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
          models.push_back(m);
      }
            
    }
    
}

//now align a pair of point clouds and return the result
//param 'cloud_source' the source pointcloud
//param 'cloud_target' is the target pointcloud
//param 'output' is the resultant aligned source pointcloud
//param 'final_transform' is the resultant transform between source and target

void pairAlign(const PointCloud::Ptr cloud_source, const PointCloud:: Ptr cloud_target, PointCloud:: Ptr output, Eigen::Matrix4f &final_transform)
{
//Downsample for consistency and speed

PointCloud::Ptr src(new PointCloud);
PointCloud::Ptr tar(new PointCloud);
pcl::VoxelGrid<PointT> grid;
if (downsample)
{
    grid.setLeafSize(0.05, 0.05, 0.05);
    grid.setInputCloud(cloud_source);
    grid.filter(*src);
    
    grid.setInputCloud(cloud_target);
    grid.filter(*tar);
   }
else
{
    src=cloud_source;
    tar=cloud_target;
}

//now we will compute surface normals and curvature
PointCoudWithNormals::Ptr points_with_normals_src (new PointCoudWithNormals);
PointCoudWithNormals::Ptr points_with_normals_tar (new PointCoudWithNormals);

pcl::NormalEstimation<PointT, PointNormalT> norm_est;
pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>()); //all hell and created only a tree pointer

//setting parameters for normal estimation
norm_est.setSearchMethod(tree);
norm_est.setKSearch(30); //probably between 30 nearest neighbours

//setting input for normal estimation and computing it for source pointcloud
norm_est.setInputCloud(cloud_source);
norm_est.compute(*points_with_normals_src);
//get a copy of the pointcloud with normals
pcl::copyPointCloud(*src, *points_with_normals_src);

//setting input for normal estimation and computing it for target pointcloud
norm_est.setInputCloud(cloud_target);
norm_est.compute(*points_with_normals_tar);
//get a copy of the pointcloud with normals
pcl::copyPointCloud(*tar, *points_with_normals_tar);

//aage kya bhai?

//instantiate out custom point representation in 'point_representaion'

MyPointRepresentation point_representaion;
// add more weight to the curvature dimension so that it will be balanced against x, y, z
float alpha[4]={0.1, 0.1, 0.1, 0.1};
point_representaion.setRescaleValues(alpha);

//align here
// the heart if the program
pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;

reg.setTransformationEpsilon(1e-6);

//setting the parameters of between the correspondences
//max distance <10 cm or 0.1 
reg.setMaxCorrespondenceDistance(0.1);

//set the point representation
reg.setPointRepresentation ( boost::make_shared<const MyPointRepresentation> (point_representaion) );
reg.setInputSource(points_with_normals_src);
reg.setInputCloud(points_with_normals_tar);

// why there is need to run the same optimization in a loop and then we have to realize the results
Eigen::Matrix4f Ti= Eigen::Matrix4f::Identity(), prev, targetToSource; //camelcase for member variables
//prev and targetToSource would be initialized to all float 0's and Ti would be assigned to Identity matrix

//get the result in a pointcloud which will be nothing but the source pointcloud
PointCoudWithNormals::Ptr reg_result = points_with_normals_src;
reg.setMaximumIterations(2);// this is doing somthing else; it's not the number of iterations

for (int i=0; i<30 ; i++)
{
    PCL_INFO("Iteration No. %d: \n", i); //this is what I love to watch..current iteration number will be displayed
    //save the cloud for the visualization purpose
    points_with_normals_src = reg_result;
    
    //Estimate.. but what?
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);
    // now accumulate the transformation function in each image
    Ti= reg.getFinalTransformation() * Ti ;
    //if the difference between this transformation and the previous one is less than the threshold, then refine the process by redcuing
    // the maximum correspondence distance
    
    // here comes fabs(): means: float absolute
    if ( fabs( ( reg.getLastIncrementalTransformation() - prev ).sum() ) < reg.getTransformationEpsilon()  )
    {
        reg.setMaxCorrespondenceDistance( reg.getMaxCorrespondenceDistance() - 0.001 );
    }
    
    prev= reg.getLastIncrementalTransformation();
    
    //visualize current state
    showCloudsRight(points_with_normals_tar, points_with_normals_src);   
    
}

//now get the transfomr from target to source
targetToSource= Ti.inverse();


//transform target back into the frame
pcl::transformPointCloud(*cloud_target, *output, targetToSource);

//again clear the display
p->removePointCloud("source");
p->removePointCloud("target");

PointCloudColorHandlerCustom<PointT> cloud_tar_h (output, 0,255, 0 ); //showing in green
PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_source, 255, 0, 0); //showing in blue

p->addPointCloud(output, cloud_tar_h, "target" , vp_2); //again vp_2 is view port 2
p->addPointCloud(cloud_source, cloud_src_h, "source", vp_2);

//ask user to get ready
PCL_INFO("Press q to continue registaration. \n");

p.spin();

p->removePointCloud("source");
p->removePointCloud("target");

//now add the soutce to hte transformed target

*output += *cloud_source;

final_transform=targetToSource;

}
