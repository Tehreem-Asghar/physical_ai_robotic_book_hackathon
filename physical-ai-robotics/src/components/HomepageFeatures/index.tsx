import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  imgPath: string; // Changed from Svg
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Foundations of Physical AI',
    imgPath: '/img/one.png', // Corrected image path
    description: (
      <>
        Start from the ground up, understanding how robots perceive, reason, and
        act in the physical world. Explore the core concepts of embodied intelligence.
      </>
    ),
  },
  {
    title: 'Master ROS 2 & Simulation',
    imgPath: '/img/two.png', // Corrected image path
    description: (
      <>
        Learn the tools of the trade, from the Robot Operating System (ROS 2) to creating
        advanced digital twins of your robots in Gazebo and NVIDIA Isaac Sim.
      </>
    ),
  },
  {
    title: 'Build AI-Powered Robots',
    imgPath: '/img/three.png', // Corrected image path
    description: (
      <>
        Integrate cutting-edge AI into your projects. Leverage hardware-accelerated perception
        and Large Language Models to build truly intelligent systems.
      </>
    ),
  },
];

function Feature({title, imgPath, description}: FeatureItem) { // Changed Svg to imgPath
  return (
    <div className={clsx('col col--4', styles.featureColumn)}>
      <div className={styles.border}>
        <div className="text--center">
          <img src={imgPath} alt={title} className={styles.featureImage} /> {/* Changed Svg to img */}
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
