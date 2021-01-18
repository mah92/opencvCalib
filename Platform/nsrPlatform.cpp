
#include <stdio.h>
#include <stdarg.h>

#include "nsrPlatform.h"

void LOGI(const char* mTAG, const char *format, ...)
{
	char logstr[400]="";
    va_list arg_list;

    va_start(arg_list, format);
	vsprintf(logstr, format, arg_list);
    va_end(arg_list);

    printf("%s", mTAG);
    printf("%s", logstr);
}
void LOGW(const char* mTAG, const char *format, ...)
{
	char logstr[400]="";
    va_list arg_list;

    va_start(arg_list, format);
	vsprintf(logstr, format, arg_list);
    va_end(arg_list);

    printf("%s", mTAG);
    printf("%s", logstr);
}
void LOGE(const char* mTAG, const char *format, ...)
{
	char logstr[400]="";
    va_list arg_list;

    va_start(arg_list, format);
	vsprintf(logstr, format, arg_list);
    va_end(arg_list);

    printf("%s", mTAG);
    printf("%s", logstr);
}
