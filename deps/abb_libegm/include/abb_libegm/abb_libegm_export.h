
#ifndef ABB_LIBEGM_EXPORT_H
#define ABB_LIBEGM_EXPORT_H

#ifdef ABB_LIBEGM_STATIC_DEFINE
#  define ABB_LIBEGM_EXPORT
#  define ABB_LIBEGM_NO_EXPORT
#else
#  ifndef ABB_LIBEGM_EXPORT
#    ifdef abb_libegm_EXPORTS
        /* We are building this library */
#      define ABB_LIBEGM_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define ABB_LIBEGM_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef ABB_LIBEGM_NO_EXPORT
#    define ABB_LIBEGM_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef ABB_LIBEGM_DEPRECATED
#  define ABB_LIBEGM_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef ABB_LIBEGM_DEPRECATED_EXPORT
#  define ABB_LIBEGM_DEPRECATED_EXPORT ABB_LIBEGM_EXPORT ABB_LIBEGM_DEPRECATED
#endif

#ifndef ABB_LIBEGM_DEPRECATED_NO_EXPORT
#  define ABB_LIBEGM_DEPRECATED_NO_EXPORT ABB_LIBEGM_NO_EXPORT ABB_LIBEGM_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef ABB_LIBEGM_NO_DEPRECATED
#    define ABB_LIBEGM_NO_DEPRECATED
#  endif
#endif

#endif /* ABB_LIBEGM_EXPORT_H */
