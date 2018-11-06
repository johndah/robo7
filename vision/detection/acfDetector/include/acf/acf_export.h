
#ifndef ACF_EXPORT_H
#define ACF_EXPORT_H

#ifdef ACF_STATIC_DEFINE
#  define ACF_EXPORT
#  define ACF_NO_EXPORT
#else
#  ifndef ACF_EXPORT
#    ifdef acf_EXPORTS
        /* We are building this library */
#      define ACF_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define ACF_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef ACF_NO_EXPORT
#    define ACF_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef ACF_DEPRECATED
#  define ACF_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef ACF_DEPRECATED_EXPORT
#  define ACF_DEPRECATED_EXPORT ACF_EXPORT ACF_DEPRECATED
#endif

#ifndef ACF_DEPRECATED_NO_EXPORT
#  define ACF_DEPRECATED_NO_EXPORT ACF_NO_EXPORT ACF_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef ACF_NO_DEPRECATED
#    define ACF_NO_DEPRECATED
#  endif
#endif

#endif /* ACF_EXPORT_H */
