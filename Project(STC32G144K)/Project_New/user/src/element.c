#include "element.h"
#include "spatial_features.h"
#include "seesaw.h"
#include "cylinder.h"
#include "huandao.h"
#include "wall.h"

/* 赛道顺序只在这里配置，允许重复同一种元素。 */
static const element_type_t element_route[] =
{
    ELEMENT_SEESAW,
    ELEMENT_CYLINDER,
    ELEMENT_HUANDAO,
    ELEMENT_WALL,
};

#define ELEMENT_ROUTE_COUNT \
    ((uint8)(sizeof(element_route) / sizeof(element_route[0])))

static volatile uint8 element_route_index = 0;
static volatile uint8 element_current_type = ELEMENT_DONE;

static void element_reset_type(element_type_t type)
{
    SpatialFeatures_Reset();

    switch (type)
    {
        case ELEMENT_SEESAW:
            Seesaw_Reset();
            break;
        case ELEMENT_CYLINDER:
            Cylinder_Reset();
            break;
        case ELEMENT_HUANDAO:
            Huandao_DetectReset();
            Huandao_Reset();
            break;
        case ELEMENT_WALL:
            Wall_Reset();
            break;
        default:
            break;
    }
}

static void element_complete(element_type_t completed_type)
{
    if ((element_type_t)element_current_type != completed_type)
        return;

    if ((uint8)(element_route_index + 1U) >= ELEMENT_ROUTE_COUNT)
    {
        element_route_index = ELEMENT_ROUTE_COUNT;
        element_current_type = ELEMENT_DONE;
        return;
    }

    element_route_index++;
    element_current_type = (uint8)element_route[element_route_index];
    element_reset_type((element_type_t)element_current_type);
}

void Element_Init(void)
{
    SpatialFeatures_Init();
    Seesaw_Init();
    Cylinder_Init();
    Huandao_DetectReset();
    Wall_Init();

    element_route_index = 0;
    element_current_type = (uint8)element_route[0];
    element_reset_type((element_type_t)element_current_type);
}

void Element_ImuUpdate(const imu_sample_t *sample)
{
    const spatial_features_t *features;

    if (sample == (const imu_sample_t *)0)
        return;

    SpatialFeatures_Update(sample);
    features = SpatialFeatures_Get();

    switch ((element_type_t)element_current_type)
    {
        case ELEMENT_SEESAW:
            (void)Seesaw_ImuUpdate(features);
            if (Seesaw_HasExited())
                element_complete(ELEMENT_SEESAW);
            break;

        case ELEMENT_CYLINDER:
            (void)Cylinder_ImuUpdate(sample->ax_g,
                                     sample->ay_g,
                                     sample->az_g,
                                     sample->gx_dps,
                                     euler.pitch);
            if (Cylinder_HasExited())
                element_complete(ELEMENT_CYLINDER);
            break;

        case ELEMENT_HUANDAO:
            if (Huandao_ConsumeExitEvent())
                element_complete(ELEMENT_HUANDAO);
            break;

        case ELEMENT_WALL:
            (void)Wall_ImuUpdate(features);
            if (Wall_HasExited())
                element_complete(ELEMENT_WALL);
            break;

        default:
            break;
    }
}

uint8 Element_AdcUpdate(void)
{
    if ((element_type_t)element_current_type == ELEMENT_CYLINDER)
    {
        (void)Cylinder_AdcUpdate();
        return 0;
    }

    if ((element_type_t)element_current_type == ELEMENT_HUANDAO)
        return Huandao_DetectUpdate();

    return 0;
}

uint8 Element_IsStraightHold(void)
{
    return (uint8)(
        (element_type_t)element_current_type == ELEMENT_HUANDAO &&
        Huandao_DetectIsStraightHold());
}

element_type_t Element_GetCurrent(void)
{
    return (element_type_t)element_current_type;
}

uint8 Element_GetRouteIndex(void)
{
    return element_route_index;
}

uint8 Element_GetRouteCount(void)
{
    return ELEMENT_ROUTE_COUNT;
}
