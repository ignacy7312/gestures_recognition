#pragma once

#include <iostream>
#include <string>

/**
 * @file BuildConfig.h
 * @brief Konfiguracja dla debug/release builds
 * 
 * Ten plik definiuje makra i funkcje które zachowują się różnie
 * w zależności od typu buildu (Debug vs Release).
 * 
 * UŻYCIE:
 * - DEBUG_LOG("message") - wypisuje tylko w debug
 * - DEBUG_ASSERT(condition, message) - assert tylko w debug
 * - RELEASE_ONLY(code) - wykonuje kod tylko w release
 * - DEBUG_ONLY(code) - wykonuje kod tylko w debug
 */

// ========== BUILD TYPE DETECTION ==========

#if defined(DEBUG_BUILD) || defined(DEBUG_MODE) || defined(_DEBUG)
    #define IS_DEBUG_BUILD 1
    #define IS_RELEASE_BUILD 0
#elif defined(RELEASE_BUILD) || defined(RELEASE_MODE) || defined(NDEBUG)
    #define IS_DEBUG_BUILD 0
    #define IS_RELEASE_BUILD 1
#else
    // Domyślnie: jeśli nic nie ustawione, traktuj jako debug
    #define IS_DEBUG_BUILD 1
    #define IS_RELEASE_BUILD 0
    #warning "Build type not specified, defaulting to DEBUG"
#endif

// ========== DEBUG LOGGING ==========

#if IS_DEBUG_BUILD
    /**
     * @brief Logowanie debug (tylko w debug build)
     * Wypisuje wiadomość do std::cout z prefiksem [DEBUG]
     */
    #define DEBUG_LOG(message) \
        std::cout << "[DEBUG] " << message << std::endl

    /**
     * @brief Logowanie debug z dodatkowym kontekstem
     * Wypisuje plik, linię i wiadomość
     */
    #define DEBUG_LOG_DETAILED(message) \
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " - " \
                  << message << std::endl

#else
    // W release: wszystkie debug logi są no-op (kompilator je usuwa)
    #define DEBUG_LOG(message) ((void)0)
    #define DEBUG_LOG_DETAILED(message) ((void)0)
    #define DEBUG_LOG_PLAYER_POS(x, y) ((void)0)
    #define DEBUG_LOG_CAMERA(camera) ((void)0)
    #define DEBUG_LOG_RENDER_STATS(visible, total, name) ((void)0)
    #define DEBUG_LOG_COLLISION_CHECKS(count) ((void)0)
#endif

// ========== ASSERTIONS ==========

#if IS_DEBUG_BUILD
    /**
     * @brief Assert sprawdzający warunek (tylko debug)
     * Jeśli warunek false, wypisuje błąd i wywołuje std::abort()
     */
    #define DEBUG_ASSERT(condition, message) \
        do { \
            if (!(condition)) { \
                std::cerr << "[ASSERT FAILED] " << __FILE__ << ":" << __LINE__ \
                          << " - " << (message) << std::endl; \
                std::abort(); \
            } \
        } while(0)
#else
    #define DEBUG_ASSERT(condition, message) ((void)0)
#endif

// ========== CONDITIONAL COMPILATION ==========

#if IS_DEBUG_BUILD
    /**
     * @brief Wykonuje kod tylko w debug build
     * Przykład: DEBUG_ONLY(std::cout << "Debug info" << std::endl;)
     */
    #define DEBUG_ONLY(code) code
    
    /**
     * @brief Kod pomijany w debug (wykonywany w release)
     */
    #define RELEASE_ONLY(code) ((void)0)
#else
    /**
     * @brief W release: debug kod jest usuwany przez kompilator
     */
    #define DEBUG_ONLY(code) ((void)0)
    
    /**
     * @brief Wykonuje kod tylko w release build
     */
    #define RELEASE_ONLY(code) code
#endif

// ========== PERFORMANCE MEASUREMENT ==========

#if IS_DEBUG_BUILD
    #include <chrono>
    
    /**
     * @brief Mierzy czas wykonania bloku kodu (tylko debug)
     * Użycie:
     * MEASURE_TIME("Collision detection", {
     *     detectCollisions();
     * });
     */
    #define MEASURE_TIME(name, code) \
        do { \
            auto start = std::chrono::high_resolution_clock::now(); \
            code \
            auto end = std::chrono::high_resolution_clock::now(); \
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); \
            std::cout << "[PERF] " << (name) << " took " << duration.count() << " µs" << std::endl; \
        } while(0)
#else
    #define MEASURE_TIME(name, code) code
#endif

// ========== BUILD INFO ==========

namespace BuildConfig {
    /**
     * @brief Zwraca czy jest to debug build
     */
    constexpr bool isDebug() {
        return IS_DEBUG_BUILD;
    }
    
    /**
     * @brief Zwraca czy jest to release build
     */
    constexpr bool isRelease() {
        return IS_RELEASE_BUILD;
    }
    
    /**
     * @brief Zwraca nazwę typu buildu jako string
     */
    inline const char* getBuildType() {
        #if IS_DEBUG_BUILD
            return "Debug";
        #else
            return "Release";
        #endif
    }
    
    /**
     * @brief Wypisuje informacje o buildzie
     */
    inline void printBuildInfo() {
        std::cout << "========== BUILD INFO ==========" << std::endl;
        std::cout << "Build type: " << getBuildType() << std::endl;
        std::cout << "Debug logging: " << (isDebug() ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "Optimizations: " << (isRelease() ? "ENABLED (-O3)" : "DISABLED") << std::endl;
        std::cout << "Assertions: " << (isDebug() ? "ENABLED" : "DISABLED") << std::endl;
        
        #ifdef __OPTIMIZE__
            std::cout << "Compiler optimization: ENABLED" << std::endl;
        #else
            std::cout << "Compiler optimization: DISABLED" << std::endl;
        #endif
        
        std::cout << "================================" << std::endl;
    }
}

// ========== INLINE HINTS ==========

#if IS_RELEASE_BUILD
    // W release: agresywne inlining dla małych funkcji
    #define FORCE_INLINE inline __attribute__((always_inline))
#else
    // W debug: normalne inline (łatwiejszy debugging)
    #define FORCE_INLINE inline
#endif

// ========== UNUSED VARIABLE SUPPRESSION ==========

/**
 * @brief Oznacza zmienną jako celowo nieużywaną (tłumi warning)
 */
#define UNUSED(x) (void)(x)

// ========== COMPATIBILITY MACROS ==========

#if IS_DEBUG_BUILD
    #define LIKELY(x) (x)     // W debug: brak branch prediction hints
    #define UNLIKELY(x) (x)
#else
    #define LIKELY(x) __builtin_expect(!!(x), 1)    // W release: hints dla kompilatora
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)
#endif
