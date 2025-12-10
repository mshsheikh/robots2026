import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-humanoid-robotics/docs',
    component: ComponentCreator('/ai-humanoid-robotics/docs', '927'),
    routes: [
      {
        path: '/ai-humanoid-robotics/docs',
        component: ComponentCreator('/ai-humanoid-robotics/docs', '47b'),
        routes: [
          {
            path: '/ai-humanoid-robotics/docs',
            component: ComponentCreator('/ai-humanoid-robotics/docs', 'e39'),
            routes: [
              {
                path: '/ai-humanoid-robotics/docs/intro',
                component: ComponentCreator('/ai-humanoid-robotics/docs/intro', '14c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week1-ros2-basics',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week1-ros2-basics', 'cb4'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
