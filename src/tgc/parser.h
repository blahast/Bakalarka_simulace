/**
 * @file
 * @brief 
 * 
 * @author David Valouch (valouch.david@fel.cvut.cz)
 * @date 2025-09-05
 * 
 * @copyright Copyright (c) 2025
 */

#pragma once
#ifndef TGC_PARSER_H_
#define TGC_PARSER_H_

#include <yaml-cpp/yaml.h>

#include "bolt/base/result.h"
#include "bolt/base/option.h"

#include <utility>

namespace tgc {

class YamlNode {
    bolt::Option<YAML::Node> _data;

public:
    YamlNode() = default;
    explicit inline YamlNode(YAML::Node&& data) 
        : _data(bolt::Some(std::forward<YAML::Node>(data))) { }
    explicit inline YamlNode(const YAML::Node& data) 
        : _data(bolt::Some(data)) { }

    enum class eError {
        ParserError,
        NonexistentValue
    };

    inline static bolt::Result<YamlNode, eError>
    make(std::string yml_file)
    {
        YAML::Node data = YAML::LoadFile(yml_file);
        if(!data.IsDefined()) return bolt::Err(eError::ParserError);

        return bolt::Ok(YamlNode(std::move(data)));
    }

    inline bool defined() const { return _data.is_some(); }

    inline size_t size() const {
        if(_data.is_none()) return 0;
        return _data.get().size();
    }

    template<typename T>
    YamlNode operator [] (const T& key) const {
        if(_data.is_none()) return YamlNode();
        auto n = _data.get()[key];
        if(!n.IsDefined()) return YamlNode();
        return YamlNode(n);
    }

    template<typename T>
    const bolt::Result<T, eError>& get() const {
        if(_data.is_none()) return bolt::Err(eError::NonexistentValue);

        T value;
        if(!YAML::convert<T>::decode(_data.get(), value))
            return bolt::Err(eError::ParserError);
        
        return bolt::Ok(std::move(value));
    }
};

} // namespace tgc

#endif // TGC_PARSER_H_