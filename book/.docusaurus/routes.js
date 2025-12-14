import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/robots2026/__docusaurus/debug',
    component: ComponentCreator('/robots2026/__docusaurus/debug', '13c'),
    exact: true
  },
  {
    path: '/robots2026/__docusaurus/debug/config',
    component: ComponentCreator('/robots2026/__docusaurus/debug/config', 'a39'),
    exact: true
  },
  {
    path: '/robots2026/__docusaurus/debug/content',
    component: ComponentCreator('/robots2026/__docusaurus/debug/content', '76e'),
    exact: true
  },
  {
    path: '/robots2026/__docusaurus/debug/globalData',
    component: ComponentCreator('/robots2026/__docusaurus/debug/globalData', '6c8'),
    exact: true
  },
  {
    path: '/robots2026/__docusaurus/debug/metadata',
    component: ComponentCreator('/robots2026/__docusaurus/debug/metadata', '503'),
    exact: true
  },
  {
    path: '/robots2026/__docusaurus/debug/registry',
    component: ComponentCreator('/robots2026/__docusaurus/debug/registry', '075'),
    exact: true
  },
  {
    path: '/robots2026/__docusaurus/debug/routes',
    component: ComponentCreator('/robots2026/__docusaurus/debug/routes', '888'),
    exact: true
  },
  {
    path: '/robots2026/docs',
    component: ComponentCreator('/robots2026/docs', 'c61'),
    routes: [
      {
        path: '/robots2026/docs',
        component: ComponentCreator('/robots2026/docs', '517'),
        routes: [
          {
            path: '/robots2026/docs',
            component: ComponentCreator('/robots2026/docs', '8aa'),
            routes: [
              {
                path: '/robots2026/docs/chat',
                component: ComponentCreator('/robots2026/docs/chat', '75b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/intro',
                component: ComponentCreator('/robots2026/docs/intro', 'a0f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week1-ros2-basics',
                component: ComponentCreator('/robots2026/docs/week1-ros2-basics', '2dc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week10-ethics-robotics',
                component: ComponentCreator('/robots2026/docs/week10-ethics-robotics', '153'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week11-applications-domains',
                component: ComponentCreator('/robots2026/docs/week11-applications-domains', '61b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week12-future-robotics',
                component: ComponentCreator('/robots2026/docs/week12-future-robotics', 'a97'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week13-conclusion',
                component: ComponentCreator('/robots2026/docs/week13-conclusion', 'b34'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week2-motion-planning',
                component: ComponentCreator('/robots2026/docs/week2-motion-planning', 'b3b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week3-perception-systems',
                component: ComponentCreator('/robots2026/docs/week3-perception-systems', '0e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week4-control-theory',
                component: ComponentCreator('/robots2026/docs/week4-control-theory', '6f9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week5-bipedal-locomotion',
                component: ComponentCreator('/robots2026/docs/week5-bipedal-locomotion', '83c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week6-manipulation-knowledge',
                component: ComponentCreator('/robots2026/docs/week6-manipulation-knowledge', '829'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week7-humanoid-hardware',
                component: ComponentCreator('/robots2026/docs/week7-humanoid-hardware', '5d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week8-ai-integration',
                component: ComponentCreator('/robots2026/docs/week8-ai-integration', '80c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/robots2026/docs/week9-human-robot-interaction',
                component: ComponentCreator('/robots2026/docs/week9-human-robot-interaction', 'd9d'),
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
    path: '/robots2026/',
    component: ComponentCreator('/robots2026/', 'f5a'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
