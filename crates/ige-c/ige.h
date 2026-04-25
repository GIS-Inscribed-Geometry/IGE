/**
 * @file ige.h
 * @brief C API for Inscribed Geometry Engine (IGE)
 *
 * This header provides a C-compatible interface for computing the largest
 * axis-aligned rectangle inscribed in a polygon.
 */

#ifndef IGE_H
#define IGE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Rectangle result structure
 */
typedef struct {
    double x_min;  ///< Minimum X coordinate
    double y_min;  ///< Minimum Y coordinate
    double x_max;  ///< Maximum X coordinate
    double y_max;  ///< Maximum Y coordinate
} IgeRectangle;

/**
 * @brief Solver configuration options
 */
typedef struct {
    double rotation_degrees;   ///< Rotation angle (0 = axis-aligned)
    int prefer_gpu;           ///< Prefer GPU when available (1=yes, 0=no)
    int force_cpu;            ///< Force CPU solver (1=yes, 0=no)
    double max_aspect_ratio;  ///< Maximum aspect ratio (0 = unlimited)
} IgeOptions;

/**
 * @brief Get default solver options
 *
 * @return Default IgeOptions structure
 */
IgeOptions ige_options_default(void);

/**
 * @brief Solve for the largest oriented inscribed rectangle
 *
 * @param coords Array of polygon coordinates [x0, y0, x1, y1, ...]
 * @param coords_len Number of elements in coords array (must be even, >= 6)
 * @param options Solver options (NULL for defaults)
 * @param result Output rectangle result
 * @return 0 on success, -1 on error
 *
 * @example
 * @code
 * IgeRectangle rect;
 * IgeOptions opts = ige_options_default();
 * double coords[] = {0, 0, 10, 0, 10, 10, 0, 10, 0, 0};
 *
 * if (ige_solve(coords, 10, &opts, &rect) == 0) {
 *     printf("Rectangle found: [%.2f, %.2f] to [%.2f, %.2f]\n",
 *            rect.x_min, rect.y_min, rect.x_max, rect.y_max);
 * }
 * @endcode
 */
int ige_solve(
    const double *coords,
    size_t coords_len,
    const IgeOptions *options,
    IgeRectangle *result
);

/**
 * @brief Calculate the area of a rectangle
 *
 * @param rect Rectangle to measure
 * @return Area (width * height)
 */
double ige_rectangle_area(const IgeRectangle *rect);

#ifdef __cplusplus
}
#endif

#endif /* IGE_H */