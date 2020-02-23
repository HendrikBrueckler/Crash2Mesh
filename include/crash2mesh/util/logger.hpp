#ifndef C2M_LOGGER_HPP
#define C2M_LOGGER_HPP

#include <ctime>
#include <iomanip>
#include <iostream>
#include <vector>

namespace c2m
{
template <typename T> class Vec2;
template <typename T> class BezierCurve;
template <typename T> class LineSegment;
template <typename T> class GuardedBezierCurve;

/**
 * @brief Use to log events of any severity level
 *
 */
class Logger
{
  public:
    Logger() = delete;
    Logger(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger& operator=(Logger&&) = delete;

    enum Level : uint8_t
    {
        NONE = 0b000000000,
        DEBUG = 0b00000001,
        INFO = 0b00000010,
        WARN = 0b00000100,
        ERROR = 0b00001000,
        FATAL = 0b00010000,
        ALL = 0b00111111
    };

    /**
     * @brief To configure messages of which severity should appear in logs.
     *        If NDEBUG is defined this is initialized to only Warnings, Error and Fatal,
     *        else everything gets logged by default.
     */
    static const Level showLvl =
#ifdef NDEBUG
        // TODO
        // static_cast<Level>(Level::ALL & ~Level::DEBUG);
        Level::ALL;
#else
        Level::ALL;
#endif

    /**
     * @brief Get the LogLevel of a message as string
     *
     */
    static std::string toString(Level msgLvl)
    {
        switch (msgLvl)
        {
        case Level::NONE:
            return "None";
        case Level::DEBUG:
            return "Debug";
        case Level::INFO:
            return "Info";
        case Level::WARN:
            return "Warning";
        case Level::ERROR:
            return "Error";
        case Level::FATAL:
            return "Fatal";
        case Level::ALL:
            return "All";
        default:
            return "Unknown";
        }
    }

    /**
     * @brief Log a message to standard output prefixed by time and severity.
     *        Message will only be logged if severity level is >= Logger::showLvl
     *
     * @param msgLvl severity level
     * @return Logger& reference to logger (for stream operator chaining)
     */
    static Logger& lout(Level msgLvl = Level::DEBUG)
    {
        static Logger l(std::cout);
        l.m_lvl = msgLvl;
        if ((l.m_lvl & showLvl) != 0)
        {
            auto t = std::time(nullptr);
            std::tm tm;
#if defined(__unix__)
            localtime_r(&t, &tm);
#elif defined(_MSC_VER)
            localtime_s(&tm, &t);
#else
            static std::mutex mtx;
            std::lock_guard<std::mutex> lock(mtx);
            tm = *std::localtime(&t);
#endif
            l << "[" << toString(msgLvl) << "] " << std::put_time(&tm, "%H:%M:%S") << ": ";
        }
        return l;
    }

    template <typename T> Logger& operator<<(const T& t)
    {
        if ((m_lvl & showLvl) != 0)
        {
            m_out << t;
        }
        return *this;
    }

    Logger& operator<<(std::ostream& (*manip)(std::ostream&))
    {
        if ((m_lvl & showLvl) != 0)
        {
            m_out << manip;
        }
        return *this;
    }

  private:
    /**
     * @brief Singleton constructor
     *
     * @param out standard output stream
     */
    Logger(std::ostream& out) : m_out(out)
    {
    }

    Level m_lvl; //!<  current severity level

    std::ostream& m_out; //!<  Which stream to log to (default is cout)
};

} // namespace c2m

#endif
