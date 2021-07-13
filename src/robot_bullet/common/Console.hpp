#ifndef ROBOT_BULLET_COMMON_CONSOLE_HPP_
#define ROBOT_BULLET_COMMON_CONSOLE_HPP_

#include <ostream>
#include <string>

/// \brief Output a message
#define dtmsg (::robot_bullet::common::colorMsg("Msg", 32))

/// \brief Output a debug message
#define dtdbg (::robot_bullet::common::colorMsg("Dbg", 36))

/// \brief Output a warning message
#define dtwarn (::robot_bullet::common::colorErr("Warning", __FILE__, __LINE__, 33))

/// \brief Output an error message
#define dterr (::robot_bullet::common::colorErr("Error", __FILE__, __LINE__, 31))

namespace robot_bullet {
    namespace common {

        /// \brief
        std::ostream& colorMsg(const std::string& _msg, int _color);

        /// \brief
        std::ostream& colorErr(
            const std::string& _msg,
            const std::string& _file,
            unsigned int _line,
            int _color);

    } // namespace common

    template <class T>
    auto operator<<(std::ostream& os, const T& t) -> decltype(t.print(os), os)
    {
        t.print(os);
        return os;
    }

} // namespace robot_bullet

#endif // ROBOT_BULLET_COMMON_CONSOLE_HPP_
