#ifndef DEF_SURFACE_DATA_H
#define DEF_SURFACE_DATA_H

#include "surfaceDatum.h"
#include "ellipticCylinder.h"
#include "trajectoryPose.h"
#include <vector>
#include <string>

namespace karst_slam {namespace scanMerging{

/** Template class for representing point resulting from a sonar measure on a surface */
template<class T>
class surfaceData
{
public:
    /** Default constructor */
    surfaceData() = default;

    /** Default destructor */
    virtual ~surfaceData() = default;

    /**
     * Add new data
     * 
     * @param datum Data to add
     */
    inline void addData(const T& datum){m_data.push_back(datum);}

     /**
     * Add new data
     * 
     * @param datum Data to add
     */
    inline void addData(T&& datum){m_data.push_back(std::move(datum));}

    /**
     * Reserve memory (as for std::vector)
     * 
     * @param size Size to reserve
     */
    inline void reserve(size_t size){m_data.reserve(size);}

    virtual void append(const surfaceData<T>& otherData){m_data.insert(m_data.end(),
                                                         otherData.m_data.begin(),
                                                         otherData.m_data.end());}

    virtual void append(surfaceData<T>&& otherData){m_data.insert(m_data.end(),
                                                   std::make_move_iterator(otherData.m_data.begin()),
                                                   std::make_move_iterator(otherData.m_data.end()));}

    /**
     * Save the data in a file
     * 
     * @param fileName File
     */
    virtual void save(const std::string& fileName) const = 0;

    /**
     * Load the data from a file
     * 
     * @param fileName File
     */
    virtual void load(const std::string& fileName) = 0;

    /**
     * Get the data
     * 
     * @return Data
     */
    inline const std::vector<T>& getData() const{return m_data;}

    /** \overload */
    inline std::vector<T>& getData() {return m_data;}

    /**
     * Number of data
     * 
     * @return Number of data
     */
    inline size_t size()const {return m_data.size();}

protected:
    std::vector<T> m_data;///< Vector of data
};

}}

#endif // DEF_SURFACE_DATA_H
