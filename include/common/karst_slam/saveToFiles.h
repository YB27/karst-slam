#ifndef DEF_SAVETOFILES_H
#define DEF_SAVETOFILES_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define SAVE_START(fileName) std::ofstream file(fileName); \
                             if(file.is_open()){ 

#define SAVE_END(fileName) file.close();} \
                 else{ std::cout << "[saveDataToFile] Failed to save file " << fileName << std::endl;} 

namespace karst_slam{namespace utils {

/** Write a pose to a file */
template<class POSE> void writePoseToFile(const POSE& pose, std::ofstream& file);

/** Save a vector of poses to a file */
template<class POSE> 
void savePosesToFile(const std::vector<POSE>& poses,
                     const std::string& fileName)
{
    SAVE_START(fileName)
    for(const POSE& pose : poses)
    {
        writePoseToFile(pose, file);
        file << "\n";
    }
    SAVE_END(fileName)
}

}}

#endif // DEF_SAVETOFILES_H