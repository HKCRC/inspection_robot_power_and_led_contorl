/*
 * Minimal stdio.h stub for clangd only.
 * Not part of the Keil build; supplies libc declarations for indexing.
 */
#ifndef _STDIO_H_STUB
#define _STDIO_H_STUB

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int _dummy; } FILE;

extern FILE *stdin;
extern FILE *stdout;
extern FILE *stderr;

#define EOF (-1)

#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

int    printf(const char *format, ...);
int    fprintf(FILE *stream, const char *format, ...);
int    sprintf(char *str, const char *format, ...);
int    snprintf(char *str, size_t size, const char *format, ...);
int    vprintf(const char *format, ...);
int    vsprintf(char *str, const char *format, ...);
int    vsnprintf(char *str, size_t size, const char *format, ...);

int    puts(const char *s);
int    putchar(int c);
int    putc(int c, FILE *stream);
int    fputc(int c, FILE *stream);
int    fputs(const char *s, FILE *stream);
int    getchar(void);
int    getc(FILE *stream);
int    fgetc(FILE *stream);
char  *fgets(char *s, int size, FILE *stream);
char  *gets(char *s);

int    scanf(const char *format, ...);
int    fscanf(FILE *stream, const char *format, ...);
int    sscanf(const char *str, const char *format, ...);

FILE  *fopen(const char *filename, const char *mode);
int    fclose(FILE *stream);
size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
int    fflush(FILE *stream);
int    fseek(FILE *stream, long offset, int whence);
long   ftell(FILE *stream);
void   rewind(FILE *stream);
int    feof(FILE *stream);
int    ferror(FILE *stream);
void   clearerr(FILE *stream);
void   perror(const char *s);
int    remove(const char *filename);
int    rename(const char *oldname, const char *newname);

#ifdef __cplusplus
}
#endif

#endif /* _STDIO_H_STUB */
