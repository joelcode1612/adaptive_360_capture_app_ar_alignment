"""Create virtual-tour capture ingestion tables."""

import sqlalchemy as sa
from alembic import op

revision = "20261028_01"
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    capture_status = sa.Enum(
        "uploading", "stitch_pending", "stitching", "stitched", "failed",
        name="capture_status",
    )
    capture_status.create(op.get_bind(), checkfirst=True)
    op.create_table(
        "virtual_tours",
        sa.Column("id", sa.Uuid(), primary_key=True),
        # No index=True: `create_index` below names these indexes, and asking
        # for both makes the second statement fail with
        # "index ix_virtual_tours_property_id already exists" on a fresh
        # database — `alembic upgrade head` could never complete.
        sa.Column("property_id", sa.Uuid(), nullable=False),
        sa.Column("owner_id", sa.Uuid(), nullable=False),
        sa.Column("expected_capture_count", sa.Integer(), nullable=False),
        sa.Column("status", capture_status, nullable=False),
        sa.Column("stitch_manifest_key", sa.String(length=512)),
        sa.Column("created_at", sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.Column("updated_at", sa.DateTime(timezone=True), server_default=sa.func.now()),
    )
    op.create_index("ix_virtual_tours_property_id", "virtual_tours", ["property_id"])
    op.create_index("ix_virtual_tours_owner_id", "virtual_tours", ["owner_id"])
    op.create_table(
        "virtual_tour_captures",
        sa.Column("id", sa.Uuid(), primary_key=True),
        sa.Column("tour_id", sa.Uuid(), sa.ForeignKey("virtual_tours.id", ondelete="CASCADE"), nullable=False),
        sa.Column("target_index", sa.Integer(), nullable=False),
        sa.Column("storage_key", sa.String(length=512), nullable=False, unique=True),
        sa.Column("sha256", sa.String(length=64), nullable=False),
        sa.Column("content_type", sa.String(length=64), nullable=False),
        sa.Column("byte_size", sa.Integer(), nullable=False),
        sa.Column("width", sa.Integer(), nullable=False),
        sa.Column("height", sa.Integer(), nullable=False),
        sa.Column("pose", sa.JSON(), nullable=False),
        sa.Column("captured_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.UniqueConstraint("tour_id", "target_index", name="uq_tour_capture_target"),
    )
    op.create_index("ix_virtual_tour_captures_tour_id", "virtual_tour_captures", ["tour_id"])


def downgrade() -> None:
    op.drop_index("ix_virtual_tour_captures_tour_id", table_name="virtual_tour_captures")
    op.drop_table("virtual_tour_captures")
    op.drop_index("ix_virtual_tours_owner_id", table_name="virtual_tours")
    op.drop_index("ix_virtual_tours_property_id", table_name="virtual_tours")
    op.drop_table("virtual_tours")
    sa.Enum(name="capture_status").drop(op.get_bind(), checkfirst=True)
