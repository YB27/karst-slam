#ifndef DEF_BASE_OBSERVABLE_H
#define DEF_BASE_OBSERVABLE_H

#include <memory>
#include <map>
#include <mutex>

namespace karst_slam{

/**
 * Base class for defining Observable/Observer pattern design.
 */ 
template <class T_Observer>
class BaseObservable
{

public:
    /** Default ctor */
    BaseObservable() = default;
    
    /** Move ctor */
    BaseObservable(BaseObservable&& obs)
    {
        std::lock_guard<std::mutex> lk(obs.m_mutex);
        m_observers = move(obs.m_observers);
    }
    
    BaseObservable(const BaseObservable& obs) = delete; 
    BaseObservable& operator=(const BaseObservable& obs) = delete;

    /** Default dtor */
    virtual ~BaseObservable(){}
    
    /** Attach an observer to this that will be notified 
     * upon events defined in derived classes
     */
    void attach(T_Observer* o)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_observers[o] = o;
    }

    /** Detach an observer  */ 
    void detach(T_Observer* o)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_observers.erase(o);
    }

    /** Detach all observers  */
    inline void detachAll(){
        std::lock_guard<std::mutex> lk(m_mutex);
        m_observers.clear();}

protected:
    std::map<T_Observer* , T_Observer*> m_observers; //!< Map containing all observers
    mutable std::mutex m_mutex; //!< mutex for thread-safety
};
}

#endif // DEF_OBSERVABLE_H
