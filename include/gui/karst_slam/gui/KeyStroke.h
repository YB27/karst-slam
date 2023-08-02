#ifndef KEYSTROKE_H
#define KEYSTROKE_H

#include <functional>
#include <map>

namespace karst_slam{namespace gui{

/** Structure defining a keystroke for the GUI */
struct keystroke
{
    keystroke(const std::string& shortcut_,
              const std::string& descr_,
              const std::function<void()>& callback_)
    {
        shortcut = shortcut_;
        descr = descr_;
        callback = callback_;
    }
    std::string shortcut; //!< shortcut of the keystroke  
    std::string descr; //!< keystroke description
    std::function<void()> callback; //!< callback called by the keystroke
};
using keystrokeMapping = std::map<std::string, keystroke>;

}} // end namespaces
#endif // KEYSTROKE_H
