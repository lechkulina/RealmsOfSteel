/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_FACTORY_H
#define ROS_FACTORY_H

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

    template<class Id, class Base, class Traits = FactoryTraits<Base> >
    class Factory {
        public:
            template<class Derived>
            bool registerClass(const Id& id) {
                if (isClassRegistered(id))
                    return false;
                creators[id] = &Traits::template create<Derived>;
                return true;
            }

            void unregisterClass(const Id& id) {
                typename CreatorMap::const_iterator iter = creators.find(id);
                if (iter != creators.end())
                    creators.erase(iter);
            }

            inline bool isClassRegistered(const Id& id) const {
                return creators.find(id) != creators.end();
            }

            Base* create(const Id& id) const {
                typename CreatorMap::const_iterator iter = creators.find(id);
                if (iter == creators.end())
                    return ROS_NULL;
                return (*(iter->second))();
            }

            inline void destroy(Base* instance) const {
                Traits::destroy(instance);
            }

            inline bool isEmpty() const {
                return creators.empty();
            }

        private:
            typedef Base* (*Creator)();
            typedef std::map<Id, Creator> CreatorMap;

            CreatorMap creators;
    };
}

#endif // ROS_FACTORY_H

