#ifndef DEF_IGL_MODEL_H
#define DEF_IGL_MODEL_H

#include <igl/readOBJ.h>
#include <mrpt/utils/CConfigFile.h>
#include <Eigen/Core>
#include <string>

namespace karst_slam{namespace simulation{
/** Handy structure to stock the vertices and faces of a 3D model loaded by the igl library */
struct iglModel
{
    iglModel() = default;
    iglModel(const mrpt::utils::CConfigFile& cfg)
    {
        std::string file = cfg.read_string("Simulation", "envModel", "");
        std::cout << "Model file : " << file << std::endl;
        loadFromOBJ(file);
    }

    inline void loadFromOBJ(const std::string& file)
    {igl::readOBJ(file, Vertices, Faces);}

    Eigen::MatrixXd Vertices; //!< Matrix of vertices (one per row)
    Eigen::MatrixXi Faces; //!< Matrix of faces (one per row, contains index of vertices)
};

}} // end namespace

#endif // DEF_IGL_MODEL_H
