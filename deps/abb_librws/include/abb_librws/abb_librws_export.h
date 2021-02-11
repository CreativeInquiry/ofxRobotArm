
#ifndef ABB_LIBRWS_EXPORT_H
#define ABB_LIBRWS_EXPORT_H

#ifdef ABB_LIBRWS_STATIC_DEFINE
#  define ABB_LIBRWS_EXPORT
#  define ABB_LIBRWS_NO_EXPORT
#else
#  ifndef ABB_LIBRWS_EXPORT
#    ifdef abb_librws_EXPORTS
        /* We are building this library */
#      define ABB_LIBRWS_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define ABB_LIBRWS_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef ABB_LIBRWS_NO_EXPORT
#    define ABB_LIBRWS_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef ABB_LIBRWS_DEPRECATED
#  define ABB_LIBRWS_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef ABB_LIBRWS_DEPRECATED_EXPORT
#  define ABB_LIBRWS_DEPRECATED_EXPORT ABB_LIBRWS_EXPORT ABB_LIBRWS_DEPRECATED
#endif

#ifndef ABB_LIBRWS_DEPRECATED_NO_EXPORT
#  define ABB_LIBRWS_DEPRECATED_NO_EXPORT ABB_LIBRWS_NO_EXPORT ABB_LIBRWS_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef ABB_LIBRWS_NO_DEPRECATED
#    define ABB_LIBRWS_NO_DEPRECATED
#  endif
#endif

#endif /* ABB_LIBRWS_EXPORT_H */
