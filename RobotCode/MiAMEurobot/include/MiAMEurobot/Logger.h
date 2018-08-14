/// \file Logger.h
/// \brief Log robot data to a csv file.
///
/// \details Logger is a wrapper made to help writing robot data to a csv log file. The user specifies a list of
///          headers and, at run time, a value for each element to log. If no value is given, the previous value is kept.
///	\note	 All functions in this header should be prefixed with logger_.
#ifndef LOGGER
#define LOGGER
	#include <glib.h>

	///< Maximum number of elements that can be logged.
	#define LOGGER_MAX_ELEMENTS 50
	typedef struct {
		GIOChannel *logFile; ///< The log file being opened.
		double currentData[LOGGER_MAX_ELEMENTS]; ///< Current data to log.
		int nElements; ///< Number of elements being logged.
	}Logger;

	/// \brief Create a logger.
	/// \details This function creates the log file, writes the header, and returns a logger struct, which can then
	///          be used to add data to the log file.
	///
	/// \param[in] filename Log filename.
	/// \param[in] logName Internal name of the log.
	/// \param[in] description Description string to add to the log first line.
	/// \param[in] headerList A comma-separated list of header. This line will be used directly for the header of the
	///                       CSV file, and will also determine the number of elements.
	///
	/// \return a Metronome struct.
	Logger logger_create(const gchar *filename, const gchar *logName, const gchar *description, const gchar *headerList);

	/// \brief Set data value to log.
	///
	/// \param[in,out] logger A logger struct.
	/// \param[in] position Column number - this should be less than nElements, else this function has no effect.
	/// \param[in] data Data value to give.
	void logger_setData(Logger *logger, int position, double data);

	/// \brief Write last data sample (i.e. content of currentData) to the csv file.
	void logger_writeLine(Logger logger);
#endif
