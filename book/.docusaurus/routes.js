import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug', 'e53'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/config',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/config', 'ec4'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/content',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/content', 'bdf'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/globalData', '230'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/metadata', '436'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/registry', 'f6a'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-humanoid-robotics/__docusaurus/debug/routes', 'ed8'),
    exact: true
  },
  {
    path: '/ai-humanoid-robotics/docs',
    component: ComponentCreator('/ai-humanoid-robotics/docs', 'd7f'),
    routes: [
      {
        path: '/ai-humanoid-robotics/docs',
        component: ComponentCreator('/ai-humanoid-robotics/docs', 'c0e'),
        routes: [
          {
            path: '/ai-humanoid-robotics/docs',
            component: ComponentCreator('/ai-humanoid-robotics/docs', '6ab'),
            routes: [
              {
                path: '/ai-humanoid-robotics/docs/intro',
                component: ComponentCreator('/ai-humanoid-robotics/docs/intro', 'cde'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week1-ros2-basics',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week1-ros2-basics', '8e2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week10-ethics-robotics',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week10-ethics-robotics', 'f39'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week11-applications-domains',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week11-applications-domains', '769'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week12-future-robotics',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week12-future-robotics', '76a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week13-conclusion',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week13-conclusion', '9ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week2-motion-planning',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week2-motion-planning', 'bfe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week3-perception-systems',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week3-perception-systems', '5f3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week4-control-theory',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week4-control-theory', 'd90'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week5-bipedal-locomotion',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week5-bipedal-locomotion', '879'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week6-manipulation-knowledge',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week6-manipulation-knowledge', '1a9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week7-humanoid-hardware',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week7-humanoid-hardware', '800'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week8-ai-integration',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week8-ai-integration', '96a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-humanoid-robotics/docs/week9-human-robot-interaction',
                component: ComponentCreator('/ai-humanoid-robotics/docs/week9-human-robot-interaction', '92f'),
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
    path: '/ai-humanoid-robotics/',
    component: ComponentCreator('/ai-humanoid-robotics/', '4b3'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
