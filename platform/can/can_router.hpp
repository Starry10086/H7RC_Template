#pragma once

#include "platform/can/can_types.hpp"

#include <array>
#include <cstdint>
#include <cstddef>

namespace librmcs::can{

struct RouterStats{
    std::size_t bound_routes{0};    // 记录绑定的路由数量
    uint32_t routed_frames{0};      // 记录成功路由的帧数
    uint32_t unhandled_frames{0};   // 记录未处理/无人领取的帧数
};

template <std::size_t MaxRoutes>
class Router final{
    static_assert(MaxRoutes > 0);

public:
    template<typename Receiver>
    bool bindExact(IdFormat format,
                   uint32_t id,
                   Receiver& receiver,
                   FrameKind kind = FrameKind::Data) noexcept{
        const uint32_t mask = format == IdFormat::Standard ? 0x7FF : 0x1FFFFFFF;
        return bindRaw(format, 
                       kind,
                       id, 
                       mask, 
                       [](void* context, const Frame& frame){
                                static_cast<Receiver*>(context)->handleCanFrame(frame);
                       }, 
              &receiver);
    }

    template<typename Receiver>
    bool bindMask(IdFormat format,
                  uint32_t id,
                  uint32_t mask,
                  Receiver& receiver,
                  FrameKind kind = FrameKind::Data) noexcept{
        return bindRaw(format,
                       kind,
                       id,
                       mask,
                       [](void* context, const Frame& frame){
                                static_cast<Receiver*>(context)->handleCanFrame(frame);
                       },
              &receiver);
    }

    bool dispatch(const Frame& frame) noexcept{
        for(std::size_t i = 0; i < route_count_; ++i){
            const Route& route = routes_[i];

            if(route.format != frame.id_format || 
               route.kind != frame.kind){
                continue;
            }

            const bool id_matches = ((frame.id ^ route.id) & route.mask) == 0U;

            if(!id_matches){
                continue;
            }

            route.handler(route.context, frame);
            ++routed_frames_;
            return true;
        }

        ++unhandled_frames_;
        return false;
    }

    RouterStats stats() const noexcept {
        return RouterStats{
            .bound_routes = route_count_,
            .routed_frames = routed_frames_,
            .unhandled_frames = unhandled_frames_,
        };
    }

private:
    using Handle = void (*)(void*, const Frame&);

    struct Route{
        IdFormat format{IdFormat::Standard};
        FrameKind kind{FrameKind::Data};
        uint32_t id{0};
        uint32_t mask{0};
        Handle handler{nullptr};    // 怎么处理
        void* context{nullptr};     // 由谁处理
    };

    bool routesOverlap(IdFormat format, FrameKind kind, uint32_t id, uint32_t mask) const noexcept{
        for(std::size_t i = 0; i < route_count_; ++i){
            const Route& route = routes_[i];

            if(route.format != format || route.kind != kind){
                continue;
            }
            const uint32_t common_mask = route.mask & mask;
            if(((route.id ^ id) & common_mask) == 0){
                return true;
            }
        }
        return false;
    }

    bool bindRaw(IdFormat format, 
                FrameKind kind, 
                uint32_t id, 
                uint32_t mask, 
                Handle handler, 
                void* context) noexcept{
        const uint32_t valid_bits = format == IdFormat::Standard ? 0x7FF : 0x1FFFFFFF;

        if( route_count_ >= MaxRoutes ||
            handler == nullptr ||
            context == nullptr ||
            mask == 0U ||
            (id & ~valid_bits) != 0U ||
            (mask & ~valid_bits) != 0U){
            return false;        
        }

        // 忽略ID中不参与匹配的位
        id &= mask;

        if(routesOverlap(format, kind, id, mask)){
            return false;
        }

        routes_[route_count_] = Route{
            .format = format,
            .kind = kind,
            .id = id,
            .mask = mask,
            .handler = handler,
            .context = context
        };

        ++route_count_;
        return true;
    }

    std::array<Route, MaxRoutes> routes_{};
    std::size_t route_count_{0};

    uint32_t routed_frames_{0};
    uint32_t unhandled_frames_{0};
};

}