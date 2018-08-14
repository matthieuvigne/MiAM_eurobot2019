#include "MiAMEurobot/Logger.h"
#include <stdio.h>

Logger logger_create(const gchar *filename, const gchar *logName, const gchar *description, const gchar *headerList)
{
	Logger logger;
	GError *err = NULL;
	// Create CSV file.
	logger.logFile = g_io_channel_new_file(filename, "w", &err);
	if(logger.logFile == NULL)
	{
		#ifdef DEBUG
			printf("Logger error when creating log file: %s\n", err->message);
		#endif
		g_error_free(err);
		return logger;
	}
	gchar *line = g_strdup_printf("Robot Log: %s,%s\n", logName, description);

	g_io_channel_write_chars(logger.logFile, line, -1, NULL, NULL);
	g_io_channel_flush(logger.logFile, NULL);
	g_free(line);

	// Determine number of elements from header list.
	logger.nElements = 0;
	gchar **split = g_strsplit (headerList, ",", -1);
	while(split[logger.nElements] != NULL)
		logger.nElements++;
	g_strfreev(split);

	// Write header list.
	g_io_channel_write_chars(logger.logFile, headerList, -1, NULL, NULL);
	g_io_channel_write_chars(logger.logFile, "\n", -1, NULL, NULL);
	g_io_channel_flush(logger.logFile, NULL);
	return logger;
}

void logger_setData(Logger *logger, int position, double data)
{
	if(position > -1 && position < logger->nElements)
		logger->currentData[position] = data;
}

void logger_writeLine(Logger logger)
{
	// If there is no file, do nothing.
	if(logger.logFile == NULL)
		return;

	// Print data line.
	for(int i = 0; i < logger.nElements - 1; i++)
	{
		gchar *data = g_strdup_printf("%f,", logger.currentData[i]);
		g_io_channel_write_chars(logger.logFile, data, -1, NULL, NULL);
		g_free(data);
	}
	// Last element: don't add trailing comma.
	gchar *data = g_strdup_printf("%f", logger.currentData[logger.nElements - 1]);
	g_io_channel_write_chars(logger.logFile, data, -1, NULL, NULL);
	g_free(data);

	g_io_channel_write_chars(logger.logFile, "\n", -1, NULL, NULL);
	g_io_channel_flush(logger.logFile, NULL);
}
