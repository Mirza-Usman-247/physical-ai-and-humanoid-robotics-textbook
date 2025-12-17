/**
 * Zod validation schemas for authentication forms.
 *
 * Provides client-side validation matching backend Pydantic models.
 */
import { z } from 'zod';

/**
 * Password validation schema.
 *
 * Requirements:
 * - Minimum 8 characters
 * - Maximum 128 characters
 * - At least 1 uppercase letter
 * - At least 1 lowercase letter
 * - At least 1 number
 */
export const passwordSchema = z
  .string()
  .min(8, 'Password must be at least 8 characters')
  .max(128, 'Password must be at most 128 characters')
  .regex(/[A-Z]/, 'Password must contain at least one uppercase letter')
  .regex(/[a-z]/, 'Password must contain at least one lowercase letter')
  .regex(/[0-9]/, 'Password must contain at least one number');

/**
 * Skill level validation schema.
 *
 * Requirements:
 * - Integer between 1 and 5 (inclusive)
 * - 1 = Beginner, 2 = Basic, 3 = Intermediate, 4 = Advanced, 5 = Expert
 */
export const skillLevelSchema = z
  .number()
  .int('Skill level must be an integer')
  .min(1, 'Skill level must be between 1 and 5')
  .max(5, 'Skill level must be between 1 and 5');

/**
 * Signup form validation schema.
 *
 * Matches backend SignupRequest Pydantic model.
 */
export const signupSchema = z.object({
  email: z
    .string()
    .email('Invalid email address')
    .min(1, 'Email is required'),

  password: passwordSchema,

  confirmPassword: z
    .string()
    .min(1, 'Please confirm your password'),

  // Skill levels (1-5 scale)
  ai_level: skillLevelSchema,
  ml_level: skillLevelSchema,
  ros_level: skillLevelSchema,
  python_level: skillLevelSchema,
  linux_level: skillLevelSchema,

  // Hardware access (optional, default: false)
  has_gpu: z.boolean().default(false),
  has_jetson: z.boolean().default(false),
  has_robot: z.boolean().default(false),
}).refine((data) => data.password === data.confirmPassword, {
  message: "Passwords don't match",
  path: ['confirmPassword'],
});

/**
 * Signin form validation schema.
 *
 * Matches backend SigninRequest Pydantic model.
 */
export const signinSchema = z.object({
  email: z
    .string()
    .email('Invalid email address')
    .min(1, 'Email is required'),

  password: z
    .string()
    .min(1, 'Password is required'),
});

/**
 * TypeScript types inferred from Zod schemas.
 */
export type SignupFormData = z.infer<typeof signupSchema>;
export type SigninFormData = z.infer<typeof signinSchema>;

/**
 * Skill level labels for UI display.
 */
export const skillLevelLabels: Record<number, string> = {
  1: 'Beginner',
  2: 'Basic',
  3: 'Intermediate',
  4: 'Advanced',
  5: 'Expert',
};

/**
 * Skill level descriptions for tooltips/help text.
 */
export const skillLevelDescriptions: Record<string, Record<number, string>> = {
  ai_level: {
    1: 'New to AI concepts',
    2: 'Familiar with basic AI terminology',
    3: 'Understanding of ML algorithms and neural networks',
    4: 'Experience with deep learning frameworks',
    5: 'Expert in AI research and deployment',
  },
  ml_level: {
    1: 'New to machine learning',
    2: 'Familiar with supervised/unsupervised learning',
    3: 'Experience with scikit-learn and model training',
    4: 'Skilled in feature engineering and model optimization',
    5: 'Expert in advanced ML techniques and production systems',
  },
  ros_level: {
    1: 'No ROS experience',
    2: 'Completed basic ROS tutorials',
    3: 'Built simple ROS applications',
    4: 'Developed complex multi-node ROS systems',
    5: 'Expert in ROS architecture and custom packages',
  },
  python_level: {
    1: 'Basic Python syntax',
    2: 'Comfortable with functions and classes',
    3: 'Proficient in Python libraries (NumPy, Pandas)',
    4: 'Advanced Python (decorators, async, metaclasses)',
    5: 'Expert in Python optimization and design patterns',
  },
  linux_level: {
    1: 'Basic command-line usage',
    2: 'Comfortable with file system and shell commands',
    3: 'Experience with package managers and scripting',
    4: 'System administration and networking',
    5: 'Expert in Linux kernel and low-level programming',
  },
};

/**
 * Hardware descriptions for checkboxes.
 */
export const hardwareDescriptions = {
  has_gpu: 'I have access to a NVIDIA GPU for deep learning training',
  has_jetson: 'I have access to NVIDIA Jetson (Nano, TX2, Xavier, Orin)',
  has_robot: 'I have access to a physical robot (humanoid, mobile, or manipulator)',
};
