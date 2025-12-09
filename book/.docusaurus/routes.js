import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug', 'acd'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/config',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/config', '81b'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/content',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/content', 'b77'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/globalData', '204'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/metadata', 'a0d'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/registry', '9ab'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/routes', '63b'),
    exact: true
  },
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
