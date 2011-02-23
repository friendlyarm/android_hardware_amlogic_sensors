/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef FOPEN_CMAKE_H__
#define FOPEN_CMAKE_H__

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

FILE *fopenCMake( const char *filename, const char *prop );
    int compareFiles( const char *filename1, const char *filename2 );

#ifdef __cplusplus
}
#endif

#endif // FOPEN_CMAKE_H__
