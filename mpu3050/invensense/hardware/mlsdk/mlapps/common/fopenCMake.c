/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: fopenCMake.c 3863 2010-10-08 22:05:31Z nroyer $
 *
 *******************************************************************************/

#include "pathConfigure.h"
#include <string.h>
#include "fopenCMake.h"

/**
 * Replacement for fopen that concatenates the
 * location of the source tree onto the filename path. It looks in 3 locations.
 * It looks first in the current directory, then it looks in "..".
 * Lastly it looks in the define UNITTEST_SOURCE_DIR which gets defined by CMake.
 * @param filename Filename relative to base of source directory
 * @param prop Second argument to fopen
 */
FILE *fopenCMake( const char *filename, const char *prop )
{
    char path[150];
    FILE *file;

    // Look first in current directory
    file = fopen(filename,prop);
    if ( file == NULL ) {
        // Now look in ".."
#ifdef WIN32
        strcpy(path,"..\\");
#else
        strcpy(path,"../");
#endif
        strcat(path,filename);
        file = fopen(path,prop);
        if ( file == NULL ) {
            // Now look in definition by CMake
            strcpy(path,PATH_SOURCE_DIR);
            strcat(path,filename);
            file = fopen(path,prop);
        }
    }
    return file;
}

/**
* Compares the data in 2 files.
* @param[in] filename1 First file
* @param[in] filename2 Second file
* @return Returns zero if files match, 1 if they don't match, 2 if filename1 could
*         not be opened, 3 if filename2 could not be opened.
*/
int compareFiles( const char *filename1, const char *filename2 )
{
    FILE *file1;
    FILE *file2;
    char buf1[64],buf2[64];
    size_t rd1,rd2,kk;

    file1 = fopenCMake( filename2, "r" );
    if ( file1 == NULL )
        return 2;
    
    file2 = fopenCMake( filename1, "r" );
    if ( file2 == NULL ) {
        fclose( file1 );
        return 3;
    }

    rd1 = 1;
    while ( rd1 != 0 ) {
        rd1 = fread( buf1, 1, 64, file1 );
        rd2 = fread( buf2, 1, 64, file2 );
        if ( rd1 != rd2 ) {
            fclose( file1 );
            fclose( file2 );
            return 1;
        }

        for (kk=0; kk<rd1; ++kk) {
            if ( buf1[kk] != buf2[kk] ) {
                fclose( file1 );
                fclose( file2 );
                return 1;
            }
        }
    }

    fclose( file1 );
    fclose( file2 );

    return 0;
}
