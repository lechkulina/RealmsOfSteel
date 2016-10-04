/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_FACTORY_H
#define ROS_FACTORY_H

#include <new>
#include <map>
#include <boost/shared_ptr.hpp>
#include <Core/Common.h>

namespace ros {

    template<class BaseClass>
    class FactoryTraits {
        public:
            template<class DerivedClass>
            static BaseClass* CreateInstance() {
                return new (std::nothrow) DerivedClass();
            }

            static void DestroyInstance(BaseClass* instance) {
                delete instance;
            }
    };

    template<class _ClassId, class _BaseClass, class _Traits = FactoryTraits<_BaseClass> >
    class Factory {
        public:
            typedef _ClassId ClassId;
            typedef _BaseClass BaseClass;
            typedef boost::shared_ptr<BaseClass> BaseClassPtr;
            typedef _Traits Traits;

            template<class DerivedClass>
            bool RegisterClass(const ClassId& id) {
                if (IsClassRegistered(id)) {
                    return false;
                }
                creators[id] = &Traits::template CreateInstance<DerivedClass>;
                return true;
            }

            void UnregisterClass(const ClassId& id) {
                CreatorsConstIter iter = creators.find(id);
                if (iter != creators.end()) {
                    creators.erase(iter);
                }
            }

            inline bool IsClassRegistered(const ClassId& id) const {
                return creators.find(id) != creators.end();
            }

            BaseClass* CreateInstance(const ClassId& id) const {
                CreatorsConstIter iter = creators.find(id);
                if (iter == creators.end()) {
                    return ROS_NULL;
                }
                return (*(iter->second))();
            }

            inline void DestroyInstance(BaseClass* instance) const {
                Traits::DestroyInstance(instance);
            }

            inline BaseClassPtr MakeShared(const ClassId& id) const {
                return BaseClassPtr(CreateInstance(id));
            }

        private:
            typedef BaseClass* (*Creator)();
            typedef std::map<ClassId, Creator> CreatorsMap;
            typedef typename CreatorsMap::iterator CreatorsIter;
            typedef typename CreatorsMap::const_iterator CreatorsConstIter;

            CreatorsMap creators;
    };

}

#endif // ROS_FACTORY_H

