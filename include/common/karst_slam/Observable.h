#ifndef DEF_OBSERVABLE_H
#define DEF_OBSERVABLE_H

#include "karst_slam/BaseObservable.h"

namespace karst_slam{class Observer;}

namespace karst_slam{
/**
 * Generic observable class. 
 * Derivated class can be used in an Observer/Observable design pattern.  
 */ 
class Observable : public BaseObservable<Observer>
{
public:
    /** Default ctor */
    Observable() = default;

    /** Move ctor */
    Observable(Observable&& obs) : BaseObservable(std::move(obs)){}

    Observable(const Observable& obs) = delete;
    Observable& operator=(const Observable& obs) = delete;

    /** Default dtor */
    virtual ~Observable(){}

    /** Every time this function is called, 
     * call a callback (with this as input) of each attached observers 
     */
    void notify();
};

/**
 * Base observer class which can be attached to any BaseObservable class
 * Derivated class can be used in an Observer/Observable design pattern.  
 */
class Observer
{
public:
    /**
     * Callback called when the observable object to which it is attached calls notify() 
     */
    virtual void updateFrom(Observable* s) = 0;
};
}
#endif // DEF_OBSERVABLE_H
