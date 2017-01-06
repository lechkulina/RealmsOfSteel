/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_FACTORY_H
#define ROS_FACTORY_H

#include <boost/regex.hpp>
#include <core/Common.h>

namespace ros {
    template<class Base>
    class FactoryTraits {
        public:
            template<class Derived>
            static Base* create() {
                return new (std::nothrow) Derived();
            }

            static void destroy(Base* instance) {
                delete instance;
            }
    };

    template<class Base, class Traits = FactoryTraits<Base> >
    class Factory {
        public:
            template<class Derived>
            bool registerClass(const boost::regex& pattern) {
                if (pattern.empty() || creators.find(pattern) != creators.end()) {
                    return false;
                }
                creators[pattern] = &Traits::template create<Derived>;
                return true;
            }

            void unregisterClass(const boost::regex& pattern) {
                typename CreatorMap::const_iterator iter = creators.find(pattern);
                if (iter != creators.end()) {
                    creators.erase(iter);
                }
            }

            inline bool isClassRegistered(const boost::regex& pattern) const {
                return creators.find(pattern) != creators.end();
            }

            inline void clear() {
                creators.clear();
            }

            inline bool isEmpty() const {
                return creators.empty();
            }

            Base* create(const char* str) const {
                for (typename CreatorMap::const_iterator iter = creators.begin(); iter != creators.end(); ++iter) {
                    if (boost::regex_match(str, iter->first)) {
                        return (*(iter->second))();
                    }
                }
                return ROS_NULL;
            }

            inline void destroy(Base* instance) const {
                Traits::destroy(instance);
            }

        private:
            typedef Base* (*CreatorCallback)();
            typedef std::map<boost::regex, CreatorCallback> CreatorMap;

            CreatorMap creators;
    };
}

#endif // ROS_FACTORY_H

